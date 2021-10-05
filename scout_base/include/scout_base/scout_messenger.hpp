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
// #include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include "scout_msgs/ScoutLightCmd.h"
#include "ugv_sdk/mobile_robot/scout_robot.hpp"

namespace westonrobot {
class ScoutMessenger {
 public:
  explicit ScoutMessenger(ros::NodeHandle *nh);
  ScoutMessenger(std::shared_ptr<ScoutRobot> scout, ros::NodeHandle *nh);

  void SetOdometryFrame(std::string frame);
  void SetBaseFrame(std::string frame);
  void SetOdometryTopicName(std::string name);
  void SetSimulationMode(int loop_rate);

  void Run();
  void Stop();

 private:
  std::shared_ptr<ScoutRobot> scout_;
  ros::NodeHandle *nh_;

  std::string odom_frame_;
  std::string base_frame_;
  std::string odom_topic_name_;

  bool simulated_robot_ = false;
  int sim_control_rate_ = 50;

  std::atomic<bool> keep_running_;
  std::mutex twist_mutex_;
  geometry_msgs::Twist current_twist_;

  ros::Publisher odom_pub_;
  ros::Publisher status_pub_;
  ros::Subscriber motion_cmd_sub_;
  ros::Subscriber light_cmd_sub_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  // speed variables
  double linear_speed_ = 0.0;
  double angular_speed_ = 0.0;

  double position_x_ = 0.0;
  double position_y_ = 0.0;
  double theta_ = 0.0;

  ros::Time last_time_;
  ros::Time current_time_;

  void SetupSubscription();

  void TwistCmdCallback(const geometry_msgs::Twist::ConstPtr &msg);
  void LightCmdCallback(const scout_msgs::ScoutLightCmd::ConstPtr &msg);
  void PublishOdometryToROS(double linear, double angular, double dt);

  void PublishStateToROS();
  void PublishSimStateToROS(double linear, double angular);
  void GetCurrentMotionCmdForSim(double &linear, double &angular);
};
}  // namespace westonrobot

#endif /* SCOUT_MESSENGER_HPP */
