#include <memory>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>

#include "scout_msgs/ScoutMotionState.h"
#include "scout_msgs/ScoutLightCmd.h"
#include "scout_msgs/ScoutCheckActuatorState.h"
#include "scout_msgs/ScoutCheckLightState.h"
#include "scout_msgs/ScoutCheckRCState.h"
#include "scout_msgs/ScoutCheckSystemState.h"

#include "ugv_sdk/scout_base.hpp"

using namespace westonrobot;

typedef enum {
  SYSTEM_MSG = 0,
  MOTION_MSG,
  LIGHT_MSG,
  RC_MSG,
  ACTUATOR_MSG
} MsgsType;

std::shared_ptr<ScoutBase> robot;

void ControlCmdCallback(const geometry_msgs::Twist::ConstPtr &msg) {
  robot->SetMotionCommand(msg->linear.x, msg->angular.z);
  ROS_INFO("received motion control cmd");
}

void LightCmdCallback(const scout_msgs::ScoutLightCmd::ConstPtr &msg) {
  ScoutLightCmd cmd;
  cmd.cmd_ctrl_allowed = msg->cmd_ctrl_allowed;
  cmd.front_mode = (LightMode)msg->front_mode;
  cmd.front_custom_value = msg->front_custom_value;
  cmd.rear_mode = (LightMode)msg->rear_mode;
  cmd.rear_custom_value = msg->rear_custom_value;
  robot->SetLightCommand(cmd);
  ROS_INFO("received light control cmd");
}

void PublishMsgTOROS(const ros::Publisher *pub, MsgsType msg_type) {
  ScoutState scout_state = robot->GetScoutState();

  switch (msg_type) {
    case SYSTEM_MSG: {
      break;
    }
    case MOTION_MSG: {
      scout_msgs::ScoutMotionState msg;
      msg.header.stamp = ros::Time::now();
      msg.linear_velocity = scout_state.motion_state.linear_velocity;
      msg.angular_velocity = scout_state.motion_state.angular_velocity;
      pub->publish(msg);
      break;
    }
    case LIGHT_MSG: {
      break;
    }
    case RC_MSG: {
      break;
    }
    case ACTUATOR_MSG: {
      break;
    }
    default:
      break;
  }
}

bool CheckActuatorState(scout_msgs::ScoutCheckActuatorState::Request &req,
                        scout_msgs::ScoutCheckActuatorState::Response &res) {
  ScoutState scout_state = robot->GetScoutState();

  uint8_t motor_id = req.motor_id;
  res.rpm = scout_state.actuator_hs_state[motor_id].rpm;
  res.current = scout_state.actuator_hs_state[motor_id].current;
  res.pulse_count = scout_state.actuator_hs_state[motor_id].pulse_count;

  res.driver_voltage = scout_state.actuator_ls_state[motor_id].driver_voltage;
  res.driver_temperature =
      scout_state.actuator_ls_state[motor_id].driver_temperature;
  res.motor_temperature =
      scout_state.actuator_ls_state[motor_id].motor_temperature;
  res.driver_state == scout_state.actuator_ls_state[motor_id].driver_state;

  ROS_INFO("Actuator %d state check request",motor_id);
  return true;
}

bool CheckLightState(scout_msgs::ScoutCheckLightState::Request &req,
                     scout_msgs::ScoutCheckLightState::Response &res) {
  ScoutState scout_state = robot->GetScoutState();

  res.cmd_ctrl_allowed = scout_state.light_state.cmd_ctrl_allowed;
  res.front_mode = scout_state.light_state.front_light.mode;
  res.front_custom_value = scout_state.light_state.front_light.custom_value;
  res.rear_mode = scout_state.light_state.rear_light.mode;
  res.rear_custom_value = scout_state.light_state.rear_light.custom_value;

  ROS_INFO("Light state check request");
  return true;
}

bool CheckRCState(scout_msgs::ScoutCheckRCState::Request &req,
                  scout_msgs::ScoutCheckRCState::Response &res) {
  ScoutState scout_state = robot->GetScoutState();

  res.swa = scout_state.rc_state.swa;
  res.swb = scout_state.rc_state.swb;
  res.swc = scout_state.rc_state.swc;
  res.swd = scout_state.rc_state.swd;

  res.stick_right_v = scout_state.rc_state.stick_right_v;
  res.stick_right_h = scout_state.rc_state.stick_right_h;
  res.stick_left_v = scout_state.rc_state.stick_left_v;
  res.stick_left_h = scout_state.rc_state.stick_left_h;
  res.var_a = scout_state.rc_state.var_a;

  ROS_INFO("Light state check request");
  return true;
}

bool CheckSystemState(scout_msgs::ScoutCheckSystemState::Request &req,
                  scout_msgs::ScoutCheckSystemState::Response &res) {
  ScoutState scout_state = robot->GetScoutState();

  res.vehicle_state = scout_state.system_state.vehicle_state;
  res.control_mode = scout_state.system_state.control_mode;
  res.battery_voltage = scout_state.system_state.battery_voltage;
  res.error_code = scout_state.system_state.error_code;

  ROS_INFO("Light state check request");
  return true;
}

int main(int argc, char **argv) {
  // setup ROS node
  ros::init(argc, argv, "scout_node");
  // ros::NodeHandle node;
  ros::NodeHandle node(""), private_node("~");
  // connect robot device
  std::string device_name;
  private_node.param<std::string>("port_name", device_name, std::string("can0"));

  robot = std::make_shared<ScoutBase>();
  robot->Connect(device_name);
  robot->EnableCommandedMode();

  // publisher
  ros::Publisher state_pub = node.advertise<scout_msgs::ScoutMotionState>(
      "/scout_motion_state", 10);

  // subscriber
  ros::Subscriber motion_cmd_sub =
      node.subscribe<geometry_msgs::Twist>("/cmd_vel", 5, ControlCmdCallback);
  ros::Subscriber light_cmd_sub = node.subscribe<scout_msgs::ScoutLightCmd>(
      "/scout_light_control", 5, LightCmdCallback);

  // servicer
  ros::ServiceServer service1 =
      node.advertiseService("check_scout_actuator", CheckActuatorState);
  ros::ServiceServer service2 =
      node.advertiseService("check_scout_light", CheckLightState);
  ros::ServiceServer service3 =
      node.advertiseService("check_scout_RC", CheckRCState);
  ros::ServiceServer service4 =
      node.advertiseService("check_scout_system", CheckSystemState);

  ros::Rate loop_rate(50);
  while (ros::ok()) {
    PublishMsgTOROS(&state_pub, MOTION_MSG);
    ros::spinOnce();
    loop_rate.sleep();
  }
}