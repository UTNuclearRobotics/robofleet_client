///////////////////////////////////////////////////////////////////////////////
//      Title     : test.cpp
//      Project   : asa_db_portal
//      Platforms : Ubuntu 64-bit
//      Copyright : Copyright© The University of Texas at Austin, 2014-2017. All rights reserved.
//
//          All files within this directory are subject to the following, unless an alternative
//          license is explicitly included within the text of each file.
//
//          This software and documentation constitute an unpublished work
//          and contain valuable trade secrets and proprietary information
//          belonging to the University. None of the foregoing material may be
//          copied or duplicated or disclosed without the express, written
//          permission of the University. THE UNIVERSITY EXPRESSLY DISCLAIMS ANY
//          AND ALL WARRANTIES CONCERNING THIS SOFTWARE AND DOCUMENTATION,
//          INCLUDING ANY WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
//          PARTICULAR PURPOSE, AND WARRANTIES OF PERFORMANCE, AND ANY WARRANTY
//          THAT MIGHT OTHERWISE ARISE FROM COURSE OF DEALING OR USAGE OF TRADE.
//          NO WARRANTY IS EITHER EXPRESS OR IMPLIED WITH RESPECT TO THE USE OF
//          THE SOFTWARE OR DOCUMENTATION. Under no circumstances shall the
//          University be liable for incidental, special, indirect, direct or
//          consequential damages or loss of profits, interruption of business,
//          or related expenses which may arise from use of software or documentation,
//          including but not limited to those resulting from defects in software
//          and/or documentation, or loss or inaccuracy of data of any kind.
//
///////////////////////////////////////////////////////////////////////////////

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt64.h>

#include <gtest/gtest.h>

class ClientTests : public ::testing::Test
{
public:
  ClientTests() :
    nh("/test/robofleet_client"),
    nh_priv_queue(nh)
  {
    nh_priv_queue.setCallbackQueue(&queue);
  }

  bool waitForReply(const rclcpp::Duration timeout = rclcpp::Duration(1.0))
  {
    const ros::Time start_time = ros::Time::now();

    while (ros::Time::now() - start_time < timeout)
    {
      if (!queue.empty())
      {
        queue.callAvailable();
        return true;
      }
    }

    return false;
  }

  ros::NodeHandle nh;
  ros::NodeHandle nh_priv_queue;
  ros::CallbackQueue queue;

protected:
  void SetUp() override
  {
  }
};

/****** TESTS ******/

TEST_F(ClientTests, ServerTime)
{
  uint64_t ping_result;
  ros::Subscriber ping_sub = nh_priv_queue.subscribe<std_msgs::UInt64>("server_time", 1,
    [&ping_result](const std_msgs::UInt64ConstPtr msg)->void {
      ping_result = msg->data;
    });
  
  ASSERT_TRUE(waitForReply());

  // we should get a positive number - the server's system time
  ASSERT_NE(ping_result, 0);

  const uint64_t prev_ping_result = ping_result;
  ASSERT_TRUE(waitForReply());

  // the server's system time should be increasing
  ASSERT_GT(ping_result, prev_ping_result);
}


TEST_F(ClientTests, StringLoopback)
{
  ros::Publisher str_pub = nh.advertise<std_msgs::String>("string_out", 1);

  std::string result_text;
  ros::Subscriber str_sub = nh_priv_queue.subscribe<std_msgs::String>("string_in", 1,
    [&result_text](const std_msgs::StringConstPtr msg)->void {
      result_text = msg->data;
    });

  // send a text message to the server
  const ros::Time time = ros::Time::now();
  const std::string out_text = "hello at time " + std::to_string(time.toSec());
  std_msgs::String msg;
  msg.data = out_text;
  str_pub.publish(msg);
  
  ASSERT_TRUE(waitForReply());

  // we should get the same text back
  ASSERT_EQ(out_text, result_text);
}


int main(int argc, char *argv[])
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "robofleet_client_test");

  ros::AsyncSpinner spinner(1);
  spinner.start();

  RCLCPP_INFO(this->get_logger(), "Starting Tests: robofleet_client client_test");
  return RUN_ALL_TESTS();
}
