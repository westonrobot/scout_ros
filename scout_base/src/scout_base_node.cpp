#include <memory>
#include <csignal>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>

#include "ugv_sdk/mobile_robot/scout_robot.hpp"
#include "scout_base/scout_messenger.hpp"

using namespace westonrobot;

std::shared_ptr<ScoutRobot> robot;
bool keep_run = true;

void DetachRobot(int signal) { keep_run = false; }

int main(int argc, char **argv) {
  // setup ROS node
  ros::init(argc, argv, "scout_node");
  ros::NodeHandle node(""), private_node("~");

  std::signal(SIGINT, DetachRobot);

  // instantiate a robot object
  robot = std::make_shared<ScoutRobot>();
  ScoutROSMessenger messenger(robot, &node);

  // fetch parameters before connecting to robot
  std::string port_name;
  private_node.param<std::string>("port_name", port_name, std::string("can0"));
  private_node.param<std::string>("odom_frame", messenger.odom_frame_,
                                  std::string("odom"));
  private_node.param<std::string>("base_frame", messenger.base_frame_,
                                  std::string("base_link"));
  private_node.param<bool>("simulated_robot", messenger.simulated_robot_,
                           false);
  private_node.param<int>("control_rate", messenger.sim_control_rate_, 50);
  private_node.param<std::string>("odom_topic_name", messenger.odom_topic_name_,
                                  std::string("odom"));

  if (!messenger.simulated_robot_) {
    // connect to robot and setup ROS subscription
    if (port_name.find("can") != std::string::npos) {
      robot->Connect(port_name);
      robot->EnableCommandedMode();
      ROS_INFO("Using CAN bus to talk with the robot");
    }
  }
  messenger.SetupSubscription();

  // publish robot state at 50Hz while listening to twist commands
  ros::Rate rate(50);
  while (keep_run) {
    if (!messenger.simulated_robot_) {
      messenger.PublishStateToROS();
    } else {
      double linear, angular;
      messenger.GetCurrentMotionCmdForSim(linear, angular);
      messenger.PublishSimStateToROS(linear, angular);
    }
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}