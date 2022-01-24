#pragma once
#include <amrl_msgs/Localization2DMsg.h>
#include <amrl_msgs/RobofleetStatus.h>
#include <amrl_msgs/RobofleetSubscription.h>
#include <amrl_msgs/VisualizationMsg.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <detection_msgs/DetectedItem.h>

#include <string>
#include <vector>

#include "RosClientNode.hpp"
#include "WebVizConstants.hpp"
#include "topic_config.hpp"

using namespace topic_config;

namespace config {
static const std::string ros_node_name = "robofleet_client";

/*****************************************************************************************
  Configure your Robotfleet Server IP Address
*/

// NRG Local Robofleet server URL
//static const std::string host_url = "ws://192.168.1.11:8080";

// UT Robofleet server URL
//static const std::string host_url = "ws://10.236.20.64:8080";

// AMRL Robofleet server URL
//static const std::string host_url = "ws://10.0.0.1:8080";

// Developement Home Local Robofleet server URL
static const std::string host_url = "ws://192.168.0.137:8080";

// ADD NEW SERVER CONFIG HERE

//static const std::string host_url = "ws://{NEW_SERVER_IP}:8080";

/*****************************************************************************************
*/

/**
 * Anti-backpressure for normal mode.
 * Uses Websocket PING/PONG protocol to gauge when server has actually received
 * a message. If true, wait for PONGs before sending more messages.
 */
static const bool wait_for_pongs = true;

/**
 * If wait_for_acks, how many more messages to send before waiting for first
 * PONG? This can be set to a value greater than 1 to compensate for network
 * latency and fully saturate available bandwidth, but if it is set too high, it
 * could cause message lag.
 */
static const uint64_t max_queue_before_waiting = 5;

/**
 * Whether to run a Websocket server instead of a client, to bypass the need
 * for a centralized instance of robofleet_server.
 */
static const bool direct_mode = false;
static const quint16 direct_mode_port =
    8080;  // what port to serve on in direct mode
static const quint64 direct_mode_bytes_per_sec =
    2048000;  // avoid network backpressure in direct mode: sets maximum upload
              // speed

/**
  * Controls the verbosity of the logging to standard output
  * 0 - Minimal Logging
  * 1 - Log for subscriptions, new message types, etc.
  * 2 - Full logging, including indication of every received message
  */
static const int verbosity = 1;


/**
 * @brief Configure which topics and types of messages the client will handle.
 *
 * You should use the .configure() method on the RosClientNode, and supply
 * either a SendLocalTopic or a ReceiveRemoteTopic config.
 *
 * To properly integrate with Robofleet, you need to run this client with a
 * ROS namespace representing the robot's name.
 *
 * Absolute topic names begin with a "/"; they will not be prefixed with the
 * current ROS namespace (robot name). Many of your local ROS nodes may publish
 * on absolute-named topics.
 * Most topics must be relative (not begin with "/") on the server side to
 * avoid name collisions between robots.
 *
 * Tips:
 * - When SENDING TO or RECEIVING FROM the server, the topic name should almost
 *   always be relative to avoid name collisions between robots.
 * - When SENDING FROM or RECEIVING TO a local topic, the topic name will often
 *   need to be absolute since many ROS nodes might not use namespaces.
 * - To send to a special webviz topic, make use of webviz_constants.
 */


static void configure_msg_types(RosClientNode& cn) {
  // Read all of the above documentation before modifying

/*****************************************************************************************
    Send Local Messages to the Robotfleet Server
    .from("/{LOCAL_ROS_MESSAGE_TOPIC_NAME}") is the topic name to publish to.
*/

  // must send to status topic to list robot in webviz
  cn.configure(SendLocalTopic<amrl_msgs::RobofleetStatus>()
                   .from("/status")
                   .to(webviz_constants::status_topic)
                   .rate_limit_hz(1));

  // must send to subscriptions topic to receive messages from other robots
  // don't drop or rate limit this topic.
  cn.configure(SendLocalTopic<amrl_msgs::RobofleetSubscription>()
                   .from("/subscriptions")
                   .to(webviz_constants::subscriptions_topic)
                   .no_drop(true));

  // send messages for webviz

  // Not needed if using GPS for position
  cn.configure(SendLocalTopic<amrl_msgs::Localization2DMsg>()
                   .from("/localization")
                   .to(webviz_constants::localization_topic)
                   .rate_limit_hz(10)
                   .priority(20));

  cn.configure(SendLocalTopic<detection_msgs::DetectedItem>()
                   .from("/detected")
                   .to(webviz_constants::detected_topic)
                   .rate_limit_hz(1));

  cn.configure(SendLocalTopic<sensor_msgs::NavSatFix>()
                    .from("/fix")
                    .to(webviz_constants::nav_sat_fix_topic)
                    .rate_limit_hz(1));

  cn.configure(SendLocalTopic<sensor_msgs::CompressedImage>()
                   .from("/stereo/left/image_raw/compressed")
                   .to(webviz_constants::compressed_image_prefix + "left")
                   .rate_limit_hz(10)
                   .priority(1));
  cn.configure(SendLocalTopic<sensor_msgs::CompressedImage>()
                   .from("/stereo/right/image_raw/compressed")
                   .to(webviz_constants::compressed_image_prefix + "right")
                   .rate_limit_hz(10)
                   .priority(1));
 
 /*****************************************************************************************
    Debugging and Development Purposes Only. 
    Receive Other Robotfleet Client Messages
    The following "ReceiveRemoteTopics" are 
    Not needed for real world robot client implementations.
    Robot clients for AugRE should only publish the topics list above with "SendLocalTopic".
*/

  // List your Robot Clients you would like to subscribe to
  std::string ClientNames[] {"2D_walrus",
                             "2D_jackelnrg",
                             "U_Regal",
                             "U_Frank"};

  // Topic Configuration
  for (auto RobotNamespace:ClientNames)
  {
  cn.configure(ReceiveRemoteTopic<amrl_msgs::Localization2DMsg>()
                   .from("/" + RobotNamespace + "/localization")
                   .to("/" + RobotNamespace + "/localization"));

  cn.configure(ReceiveRemoteTopic<amrl_msgs::RobofleetStatus>()
                   .from("/" + RobotNamespace + "/status")
                   .to("/" + RobotNamespace + "/status"));

  cn.configure(ReceiveRemoteTopic<detection_msgs::DetectedItem>()
                   .from("/" + RobotNamespace + "/detected")
                   .to("/" + RobotNamespace + "/detected"));
                   
  cn.configure(ReceiveRemoteTopic<sensor_msgs::NavSatFix>()
                    .from("/" + RobotNamespace + "/NavSatFix")
                    .to("/" + RobotNamespace + "/NavSatFix"));
  };

  /*****************************************************************************************
  */
}
}  // namespace config
