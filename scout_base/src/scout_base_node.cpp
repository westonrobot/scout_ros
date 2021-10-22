#include <memory>
#include <csignal>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>

#include "ugv_sdk/mobile_robot/scout_robot.hpp"
#include "ugv_sdk/utilities/protocol_detector.hpp"

#include "scout_base/scout_messenger.hpp"

using namespace westonrobot;

int main(int argc, char **argv) {
  // setup ROS node
  ros::init(argc, argv, "scout_node");
  ros::NodeHandle node(""), private_node("~");

  // fetch parameters before connecting to robot
  std::string port_name;
  std::string odom_frame;
  std::string base_frame;
  std::string odom_topic_name;
  bool is_omni_wheel = false;
  bool is_simulated = false;
  int sim_rate = 50;
  bool is_scout_mini = false;

  private_node.param<std::string>("port_name", port_name, std::string("can0"));
  private_node.param<std::string>("odom_frame", odom_frame,
                                  std::string("odom"));
  private_node.param<std::string>("base_frame", base_frame,
                                  std::string("base_link"));
  private_node.param<bool>("is_omni_wheel", is_omni_wheel, false);
  private_node.param<bool>("simulated_robot", is_simulated, false);
  private_node.param<int>("control_rate", sim_rate, 50);
  private_node.param<std::string>("odom_topic_name", odom_topic_name,
                                  std::string("odom"));
  private_node.param<bool>("is_scout_mini", is_scout_mini, false);

  if (is_scout_mini) {
    ROS_INFO("Robot base: Scout Mini");
  } else {
    ROS_INFO("Robot base: Scout");
  }

  if (!is_omni_wheel) {
    // instantiate a robot object
    std::shared_ptr<ScoutRobot> robot;

    ProtocolDectctor detector;
    if (detector.Connect(port_name)) {
      auto proto = detector.DetectProtocolVersion(5);
      if (proto == ProtocolVersion::AGX_V1) {
        std::cout << "Detected protocol: AGX_V1" << std::endl;
        robot = std::make_shared<ScoutRobot>(ProtocolVersion::AGX_V1,
                                             is_scout_mini);
      } else if (proto == ProtocolVersion::AGX_V2) {
        std::cout << "Detected protocol: AGX_V2" << std::endl;
        robot = std::make_shared<ScoutRobot>(ProtocolVersion::AGX_V2,
                                             is_scout_mini);
      } else {
        std::cout << "Detected protocol: UNKONWN" << std::endl;
        return -1;
      }
    } else {
      return -1;
    }

    // instantiate a ROS messenger
    std::unique_ptr<ScoutMessenger<ScoutRobot>> messenger =
        std::unique_ptr<ScoutMessenger<ScoutRobot>>(
            new ScoutMessenger<ScoutRobot>(robot, &node));

    messenger->SetOdometryFrame(odom_frame);
    messenger->SetBaseFrame(base_frame);
    messenger->SetOdometryTopicName(odom_topic_name);
    if (is_simulated) messenger->SetSimulationMode();

    // connect to robot and setup ROS subscription
    if (port_name.find("can") != std::string::npos) {
      if (robot->Connect(port_name)) {
        robot->EnableCommandedMode();
        ROS_INFO("Using CAN bus to talk with the robot");
      } else {
        ROS_INFO("Failed to connect to the robot CAN bus");
        return -1;
      }
    }

    messenger->Run();
  } else {
    // instantiate a robot object
    std::shared_ptr<ScoutMiniOmniRobot> robot;

    ProtocolDectctor detector;
    detector.Connect(port_name);
    auto proto = detector.DetectProtocolVersion(5);
    if (proto == ProtocolVersion::AGX_V1) {
      std::cout << "Detected protocol: AGX_V1" << std::endl;
      robot = std::unique_ptr<ScoutMiniOmniRobot>(
          new ScoutMiniOmniRobot(ProtocolVersion::AGX_V1));

    } else if (proto == ProtocolVersion::AGX_V2) {
      std::cout << "Detected protocol: AGX_V2" << std::endl;
      robot = std::unique_ptr<ScoutMiniOmniRobot>(
          new ScoutMiniOmniRobot(ProtocolVersion::AGX_V2));
    } else {
      std::cout << "Detected protocol: UNKONWN" << std::endl;
      return -1;
    }

    // instantiate a ROS messenger
    std::unique_ptr<ScoutMessenger<ScoutMiniOmniRobot>> messenger =
        std::unique_ptr<ScoutMessenger<ScoutMiniOmniRobot>>(
            new ScoutMessenger<ScoutMiniOmniRobot>(robot, &node));

    messenger->SetOdometryFrame(odom_frame);
    messenger->SetBaseFrame(base_frame);
    messenger->SetOdometryTopicName(odom_topic_name);
    if (is_simulated) messenger->SetSimulationMode();

    // connect to robot and setup ROS subscription
    if (port_name.find("can") != std::string::npos) {
      if (robot->Connect(port_name)) {
        robot->EnableCommandedMode();
        ROS_INFO("Using CAN bus to talk with the robot");
      } else {
        ROS_INFO("Failed to connect to the robot CAN bus");
        return -1;
      }
    }

    messenger->Run();
  }

  return 0;
}