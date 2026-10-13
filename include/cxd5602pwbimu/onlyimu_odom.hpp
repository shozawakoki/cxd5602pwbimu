/*
  Copyright 2025 shunsugar <shin.ioaoi@gmail.com>

  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at

      http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.
*/

#ifndef ONLYIMU_ODOM_HPP
#define ONLYIMU_ODOM_HPP

#include <array>
#include <memory>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_ros/transform_broadcaster.h>

class OnlyImuOdomNode : public rclcpp::Node
{
public:
  OnlyImuOdomNode();

private:
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void publishOdometry(
    const sensor_msgs::msg::Imu::SharedPtr msg,
    const tf2::Quaternion & orientation,
    const tf2::Vector3 & acceleration_world);

  static double vectorNorm(const tf2::Vector3 & v);
  static tf2::Vector3 applyDeadband(const tf2::Vector3 & v, double threshold);

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  std::string imu_topic_;
  std::string odom_topic_;
  std::string odom_header_frame_;
  std::string odom_child_frame_;
  std::string tf_header_frame_;
  std::string tf_child_frame_;

  bool publish_tf_;
  bool planar_mode_;
  bool nonholonomic_constraint_;
  bool zupt_enabled_;
  bool debug_;

  int calibration_samples_;
  double gravity_magnitude_;
  double accel_lpf_alpha_;
  double accel_deadband_;
  double stationary_accel_threshold_;
  double stationary_gyro_threshold_;
  double stationary_time_threshold_;
  double max_dt_;

  bool have_prev_stamp_;
  bool calibrated_;
  bool have_prev_accel_;
  bool have_filtered_accel_;

  rclcpp::Time prev_stamp_;

  int calibration_count_;
  tf2::Vector3 accel_bias_sum_body_;
  tf2::Vector3 accel_bias_body_;

  tf2::Vector3 position_world_;
  tf2::Vector3 velocity_world_;
  tf2::Vector3 prev_accel_world_;
  tf2::Vector3 filtered_accel_world_;

  double stationary_time_;
};

#endif  // ONLYIMU_ODOM_HPP
