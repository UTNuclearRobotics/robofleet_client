#include <rclcpp/rclcpp.hpp>

#include <QObject>
#include <QtCore/QCoreApplication>

#include <cstdint>
#include <iostream>
#include <string>
#include <thread>

#include "MessageScheduler.hpp"
#include "RosClientNode.hpp"
#include "WsClient.hpp"
#include "WsServer.hpp"

#include <yaml-cpp/yaml.h>

struct ProgramParams
{
  std::string host_url;
  bool wait_for_pongs;
  uint64_t max_queue_before_waiting;
  quint16 direct_mode_port;
  quint64 direct_mode_bytes_per_sec;
  RosClientNode::Verbosity verbosity;
  int spin_threads;
};

bool loadYAMLParams(const YAML::Node& root,
                    ProgramParams& out_params,
                    rclcpp::Logger& logger);

void connect_server(WsServer& ws_server,
                    std::shared_ptr<RosClientNode> ros_node);

void connect_client(WsClient& ws_client,
                    std::shared_ptr<RosClientNode> ros_node,
                    MessageScheduler& scheduler);

int main(int argc, char** argv) {
  QCoreApplication qapp(argc, argv);

  rclcpp::init(argc, argv);

  rclcpp::Logger logger = rclcpp::get_logger("robofleet_client_main_logger");

  // check args
  if (argc != 2) {
    RCLCPP_FATAL(logger, "usage: robofleet_client client config_file\n"
              "\tconfig_file: The program config file. See robofleet_client/cfg/example.yaml");

    return 1;
  }

  const std::string cfg_file(argv[1]);
  YAML::Node root;

  // load YAML params file
  try {
    root = YAML::LoadFile(cfg_file);
  } catch (const YAML::BadFile& e) {
    RCLCPP_FATAL(logger, "Failed to open config file %s.\n%s",
              cfg_file.c_str(), e.what());

    return false;
  }

  // parse YAML params
  ProgramParams params;
  if (!loadYAMLParams(root,
                      params,
                      logger)) {
    return 2;
  }

  const bool use_direct_mode = params.host_url.empty();  

  rclcpp::executors::MultiThreadedExecutor executor;

  // start websocket
  if (use_direct_mode) {
    // Websocket server
    WsServer ws_server(params.direct_mode_port,
                       params.direct_mode_bytes_per_sec);

    // launch ROS node
    std::shared_ptr<RosClientNode> ros_node = std::make_shared<RosClientNode>(params.verbosity, ws_server);

    connect_server(ws_server, ros_node);

    if (!ros_node->configure(root)) {
      return 3;
    }

    std::thread qt_thread([&ros_node] {
      rclcpp::spin(ros_node);
      rclcpp::shutdown(); 
    });
    
    return qapp.exec();
  } else {
    MessageScheduler scheduler(params.max_queue_before_waiting);

    // launch ROS node
    std::shared_ptr<RosClientNode> ros_node = std::make_shared<RosClientNode>(params.verbosity, scheduler);

    // Websocket client
    WsClient ws_client{QString::fromStdString(params.host_url)};

    connect_client(ws_client, ros_node, scheduler);

    if (!ros_node->configure(root)) {
      return 3;
    }

    std::thread qt_thread([&ros_node] {
      rclcpp::spin(ros_node);
      rclcpp::shutdown(); 
    });

    return qapp.exec();
  }

  return 0;
}

bool loadYAMLParams(const YAML::Node& root,
                    ProgramParams& out_params,
                    rclcpp::Logger& logger)
{
  try {
    const YAML::Node params_yaml = root["params"];
    const bool use_direct_mode = params_yaml["direct_mode_port"] || params_yaml["direct_mode_bytes_per_sec"];

    out_params.wait_for_pongs = params_yaml["wait_for_pongs"].as<bool>();

    const int max_queue_value = params_yaml["max_queue_before_waiting"].as<int>();
    if (max_queue_value < 0) {
      RCLCPP_FATAL(logger, "Invalid value %d for param max_queue_before_waiting. "
                "Must be positive.", max_queue_value);

      return false;
    }
    out_params.max_queue_before_waiting = max_queue_value;

    const int spin_threads = params_yaml["spin_threads"].as<int>();
    if (spin_threads < 0) {
      RCLCPP_FATAL(logger, "Invalid value %d for param spin_threads. "
                "Must be positive.", spin_threads);

      return false;
    }
    out_params.spin_threads = spin_threads;

    /** DIRECT MODE PARAMS **/
    if (use_direct_mode) {
      const int port_value = params_yaml["direct_mode_port"].as<int>();
      if (port_value < 0) {
        RCLCPP_FATAL(logger, "Invalid value %d for param direct_mode_port. "
                  "Must be positive.", port_value);

        return false;
      }
      out_params.direct_mode_port = port_value;

      const int byte_rate_value = params_yaml["direct_mode_bytes_per_sec"].as<int>();
      if (byte_rate_value < 0) {
        RCLCPP_FATAL(logger, "Invalid value %d for param direct_mode_bytes_per_sec. "
                  "Must be positive.", byte_rate_value);

        return false;
      }
      out_params.direct_mode_bytes_per_sec = byte_rate_value;
    }
    else {
      out_params.host_url = params_yaml["host_url"].as<std::string>();
    }

    const int verbosity_value = params_yaml["verbosity"].as<int>();
    if (verbosity_value < 0 || verbosity_value > (int)RosClientNode::Verbosity::ALL) {
      RCLCPP_FATAL(logger, "Invalid value %d for param verbosity. "
                "Must be positive and not more than %d.",
                verbosity_value, (int)RosClientNode::Verbosity::ALL);

      return false;
    }
    out_params.verbosity = RosClientNode::Verbosity(verbosity_value);

  } catch (const YAML::InvalidNode& e) {
    RCLCPP_FATAL(logger, "%s", e.what());
    return false;
  }

  return true;
}

void connect_client(WsClient& ws_client,
                    std::shared_ptr<RosClientNode> ros_node,
                    MessageScheduler& scheduler) {
  // run scheduler
  QObject::connect(&ws_client,
                   &WsClient::backpressure_update,
                   &scheduler,
                   &MessageScheduler::backpressure_update);
  // send scheduled message
  QObject::connect(&scheduler,
                   &MessageScheduler::scheduled,
                   &ws_client,
                   &WsClient::send_message);

  // receive
  QObject::connect(&ws_client,
                   &WsClient::message_received,
                   ros_node.get(),
                   &RosClientNode::routeMessageToHandlers);

  QObject::connect(
      &ws_client,
      &WsClient::connected,
      ros_node.get(),
      &RosClientNode::sendSubscriptionMsg);
}

void connect_server(WsServer& ws_server,
                    std::shared_ptr<RosClientNode> ros_node) {

  // receive
  QObject::connect(
      &ws_server,
      &WsServer::binary_message_received,
      ros_node.get(),
      &RosClientNode::routeMessageToHandlers);
}