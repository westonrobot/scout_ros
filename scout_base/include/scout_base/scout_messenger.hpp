/*
 * scout_messenger.hpp
 *
 * Created on: Jun 14, 2019 10:24
 * Description:
 *
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#ifndef SCOUT_MESSENGER_HPP
#define SCOUT_MESSENGER_HPP

#include <string>
#include <mutex>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf2_ros/transform_broadcaster.h>

#include "scout_msgs/ScoutStatus.h"
#include "scout_msgs/ScoutLightCmd.h"

#include "ugv_sdk/mobile_robot/scout_robot.hpp"

namespace westonrobot {
template <typename ScoutType>
class ScoutMessenger {
 public:
  ScoutMessenger(std::shared_ptr<ScoutType> scout, ros::NodeHandle *nh)
      : scout_(scout), nh_(nh) {}

  void SetOdometryFrame(std::string frame) { odom_frame_ = frame; }
  void SetBaseFrame(std::string frame) { base_frame_ = frame; }
  void SetOdometryTopicName(std::string name) { odom_topic_name_ = name; }

  void SetSimulationMode() { simulated_robot_ = true; }

  void SetupSubscription() {
    // odometry publisher
    odom_pub_ = nh_->advertise<nav_msgs::Odometry>(odom_topic_name_, 50);
    status_pub_ = nh_->advertise<scout_msgs::ScoutStatus>("/scout_status", 10);

    // cmd subscriber
    motion_cmd_sub_ = nh_->subscribe<geometry_msgs::Twist>(
        "/cmd_vel", 5, &ScoutMessenger::TwistCmdCallback, this);
    light_cmd_sub_ = nh_->subscribe<scout_msgs::ScoutLightCmd>(
        "/scout_light_control", 5, &ScoutMessenger::LightCmdCallback, this);
  }

  void Run() {
    SetupSubscription();

    // publish robot state at 50Hz while listening to twist commands
    ros::Rate rate(50);
    while (ros::ok()) {
      PublishStateToROS();
      ros::spinOnce();
      rate.sleep();
    }
  }

  void Update() {
    PublishStateToROS();
    ros::spinOnce();
  }

 private:
  std::shared_ptr<ScoutType> scout_;
  ros::NodeHandle *nh_;

  std::string odom_frame_;
  std::string base_frame_;
  std::string odom_topic_name_;

  bool simulated_robot_ = false;

  std::mutex twist_mutex_;
  geometry_msgs::Twist current_twist_;

  ros::Publisher odom_pub_;
  ros::Publisher status_pub_;
  ros::Subscriber motion_cmd_sub_;
  ros::Subscriber light_cmd_sub_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  // speed variables
  double position_x_ = 0.0;
  double position_y_ = 0.0;
  double theta_ = 0.0;

  ros::Time last_time_;
  ros::Time current_time_;

  void TwistCmdCallback(const geometry_msgs::Twist::ConstPtr &msg) {
    SetScoutMotionCommand(scout_, msg);
    ROS_INFO("Cmd received:%f, %f\n", msg->linear.x, msg->angular.z);
  }

  template <typename T,
            std::enable_if_t<!std::is_base_of<ScoutMiniOmniRobot, T>::value,
                             bool> = true>
  void SetScoutMotionCommand(std::shared_ptr<T> base,
                             const geometry_msgs::Twist::ConstPtr &msg) {
    base->SetMotionCommand(msg->linear.x, msg->angular.z);
  }

  template <typename T,
            std::enable_if_t<std::is_base_of<ScoutMiniOmniRobot, T>::value,
                             bool> = true>
  void SetScoutMotionCommand(std::shared_ptr<T> base,
                             const geometry_msgs::Twist::ConstPtr &msg) {
    base->SetMotionCommand(msg->linear.x, msg->angular.z, msg->linear.y);
  }

  void LightCmdCallback(const scout_msgs::ScoutLightCmd::ConstPtr &msg) {
    if (!simulated_robot_) {
      if (msg->cmd_ctrl_allowed) {
        AgxLightMode f_mode;
        uint8_t f_value;

        switch (msg->front_mode) {
          case scout_msgs::ScoutLightCmd::LIGHT_CONST_OFF: {
            f_mode = AgxLightMode::CONST_OFF;
            break;
          }
          case scout_msgs::ScoutLightCmd::LIGHT_CONST_ON: {
            f_mode = AgxLightMode::CONST_ON;
            break;
          }
          case scout_msgs::ScoutLightCmd::LIGHT_BREATH: {
            f_mode = AgxLightMode::BREATH;
            break;
          }
          case scout_msgs::ScoutLightCmd::LIGHT_CUSTOM: {
            f_mode = AgxLightMode::CUSTOM;
            f_value = msg->front_custom_value;
            break;
          }
        }
        scout_->SetLightCommand(f_mode, f_value, AgxLightMode::CONST_ON, 0);
      } else {
        scout_->DisableLightControl();
      }
    } else {
      std::cout << "simulated robot received light control cmd" << std::endl;
    }
  }

  void PublishOdometryToROS(const MotionStateMessage &msg, double dt) {
    auto ctime = ros::Time::now();

    // perform numerical integration to get an estimation of pose
    double linear_speed = msg.linear_velocity;
    double angular_speed = msg.angular_velocity;
    double lateral_speed = 0;

    if (std::is_base_of<ScoutMiniOmniRobot, ScoutType>::value) {
      lateral_speed = msg.lateral_velocity;
    } else {
      lateral_speed = 0;
    }

    double d_x =
        (linear_speed * std::cos(theta_) - lateral_speed * std::sin(theta_)) *
        dt;
    double d_y =
        (linear_speed * std::sin(theta_) + lateral_speed * std::cos(theta_)) *
        dt;
    double d_theta = angular_speed * dt;

    position_x_ += d_x;
    position_y_ += d_y;
    theta_ += d_theta;

    geometry_msgs::Quaternion odom_quat =
        tf::createQuaternionMsgFromYaw(theta_);

    // publish tf transformation
    geometry_msgs::TransformStamped tf_msg;
    tf_msg.header.stamp = ctime;
    tf_msg.header.frame_id = odom_frame_;
    tf_msg.child_frame_id = base_frame_;

    tf_msg.transform.translation.x = position_x_;
    tf_msg.transform.translation.y = position_y_;
    tf_msg.transform.translation.z = 0.0;
    tf_msg.transform.rotation = odom_quat;

    tf_broadcaster_.sendTransform(tf_msg);

    // publish odometry and tf messages
    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = ctime;
    odom_msg.header.frame_id = odom_frame_;
    odom_msg.child_frame_id = base_frame_;

    odom_msg.pose.pose.position.x = position_x_;
    odom_msg.pose.pose.position.y = position_y_;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation = odom_quat;

    odom_msg.twist.twist.linear.x = linear_speed;
    odom_msg.twist.twist.linear.y = lateral_speed;
    odom_msg.twist.twist.angular.z = angular_speed;

    odom_pub_.publish(odom_msg);
  }

  void PublishStateToROS() {
    current_time_ = ros::Time::now();
    double dt = (current_time_ - last_time_).toSec();

    static bool init_run = true;
    if (init_run) {
      last_time_ = current_time_;
      init_run = false;
      return;
    }

    auto state = scout_->GetRobotState();

    // publish scout state message
    scout_msgs::ScoutStatus status_msg;

    status_msg.header.stamp = current_time_;

    status_msg.linear_velocity = state.motion_state.linear_velocity;
    status_msg.angular_velocity = state.motion_state.angular_velocity;

    status_msg.vehicle_state = state.system_state.vehicle_state;
    status_msg.control_mode = state.system_state.control_mode;
    status_msg.error_code = state.system_state.error_code;
    status_msg.battery_voltage = state.system_state.battery_voltage;

    auto actuator = scout_->GetActuatorState();

    for (int i = 0; i < 4; ++i) {
      // actuator_hs_state
      uint8_t motor_id = actuator.actuator_hs_state[i].motor_id;

      status_msg.actuator_states[motor_id].motor_id = motor_id;
      status_msg.actuator_states[motor_id].rpm =
          actuator.actuator_hs_state[i].rpm;
      status_msg.actuator_states[motor_id].current =
          actuator.actuator_hs_state[i].current;
      status_msg.actuator_states[motor_id].pulse_count =
          actuator.actuator_hs_state[i].pulse_count;

      // actuator_ls_state
      motor_id = actuator.actuator_ls_state[i].motor_id;

      status_msg.actuator_states[motor_id].motor_id = motor_id;
      status_msg.actuator_states[motor_id].driver_voltage =
          actuator.actuator_ls_state[i].driver_voltage;
      status_msg.actuator_states[motor_id].driver_temperature =
          actuator.actuator_ls_state[i].driver_temp;
      status_msg.actuator_states[motor_id].motor_temperature =
          actuator.actuator_ls_state[i].motor_temp;
      status_msg.actuator_states[motor_id].driver_state =
          actuator.actuator_ls_state[i].driver_state;
    }

    status_msg.light_control_enabled = state.light_state.enable_cmd_ctrl;
    status_msg.front_light_state.mode = state.light_state.front_light.mode;
    status_msg.front_light_state.custom_value =
        state.light_state.front_light.custom_value;
    status_msg.rear_light_state.mode = state.light_state.rear_light.mode;
    status_msg.rear_light_state.custom_value =
        state.light_state.rear_light.custom_value;
    status_pub_.publish(status_msg);

    // publish odometry and tf
    PublishOdometryToROS(state.motion_state, dt);

    // record time for next integration
    last_time_ = current_time_;
  }
};
}  // namespace westonrobot

#endif /* SCOUT_MESSENGER_HPP */
