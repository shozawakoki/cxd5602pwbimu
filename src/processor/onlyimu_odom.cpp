/*
  IMU-only odometry for cxd5602pwbimu.

  The node uses no wheel odometry. Translation is estimated by double
  integration of IMU linear acceleration, and orientation is taken from the
  input IMU quaternion (normally Madgwick + initial yaw correction).
*/

#include "cxd5602pwbimu/onlyimu_odom.hpp"

#include <algorithm>
#include <cmath>
#include <functional>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

OnlyImuOdomNode::OnlyImuOdomNode()
: Node("onlyimu_odom"),
  publish_tf_(false),
  planar_mode_(true),
  nonholonomic_constraint_(false),
  zupt_enabled_(false),
  debug_(false),
  calibration_samples_(960),
  gravity_magnitude_(9.80665),
  accel_lpf_alpha_(0.15),
  accel_deadband_(0.03),
  stationary_accel_threshold_(0.10),
  stationary_gyro_threshold_(0.02),
  stationary_time_threshold_(0.25),
  max_dt_(0.10),
  have_prev_stamp_(false),
  calibrated_(false),
  have_prev_accel_(false),
  have_filtered_accel_(false),
  calibration_count_(0),
  accel_bias_sum_body_(0.0, 0.0, 0.0),
  accel_bias_body_(0.0, 0.0, 0.0),
  position_world_(0.0, 0.0, 0.0),
  velocity_world_(0.0, 0.0, 0.0),
  prev_accel_world_(0.0, 0.0, 0.0),
  filtered_accel_world_(0.0, 0.0, 0.0),
  stationary_time_(0.0)
{
  imu_topic_ = this->declare_parameter<std::string>("imu_topic", "/imu/spresense/inityaw");
  odom_topic_ = this->declare_parameter<std::string>("odom_topic", "/odom/onlyimu");

  odom_header_frame_ = this->declare_parameter<std::string>("odom_header_frame", "odom");
  odom_child_frame_ = this->declare_parameter<std::string>("odom_child_frame", "base_footprint");
  tf_header_frame_ = this->declare_parameter<std::string>("TF_header_frame", "odom");
  tf_child_frame_ = this->declare_parameter<std::string>("TF_child_frame", "base_footprint");

  publish_tf_ = this->declare_parameter<bool>("publish_TF", false);
  planar_mode_ = this->declare_parameter<bool>("planar_mode", true);
  nonholonomic_constraint_ = this->declare_parameter<bool>("nonholonomic_constraint", false);
  // WARNING: an IMU alone cannot distinguish standstill from constant-velocity
  // straight motion. ZUPT is therefore opt-in, not enabled by default.
  zupt_enabled_ = this->declare_parameter<bool>("zupt_enabled", false);
  debug_ = this->declare_parameter<bool>("debug", false);

  calibration_samples_ = this->declare_parameter<int>("calibration_samples", 960);
  gravity_magnitude_ = this->declare_parameter<double>("gravity_magnitude", 9.80665);
  accel_lpf_alpha_ = this->declare_parameter<double>("accel_lpf_alpha", 0.15);
  accel_deadband_ = this->declare_parameter<double>("accel_deadband", 0.03);
  stationary_accel_threshold_ =
    this->declare_parameter<double>("stationary_accel_threshold", 0.10);
  stationary_gyro_threshold_ =
    this->declare_parameter<double>("stationary_gyro_threshold", 0.02);
  stationary_time_threshold_ =
    this->declare_parameter<double>("stationary_time_threshold", 0.25);
  max_dt_ = this->declare_parameter<double>("max_dt", 0.10);

  calibration_samples_ = std::max(calibration_samples_, 1);
  accel_lpf_alpha_ = std::max(0.0, std::min(accel_lpf_alpha_, 1.0));
  accel_deadband_ = std::max(accel_deadband_, 0.0);
  stationary_accel_threshold_ = std::max(stationary_accel_threshold_, 0.0);
  stationary_gyro_threshold_ = std::max(stationary_gyro_threshold_, 0.0);
  stationary_time_threshold_ = std::max(stationary_time_threshold_, 0.0);
  max_dt_ = std::max(max_dt_, 1.0e-3);

  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    imu_topic_, 10,
    std::bind(&OnlyImuOdomNode::imuCallback, this, std::placeholders::_1));
  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 10);

  if (publish_tf_) {
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  }

  RCLCPP_INFO(this->get_logger(), "Initialized onlyimu_odom node.");
  RCLCPP_INFO(this->get_logger(), "IMU input: %s", imu_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "Odometry output: %s", odom_topic_.c_str());
  RCLCPP_INFO(
    this->get_logger(),
    "Keep the robot stationary for the first %d IMU samples for accelerometer calibration.",
    calibration_samples_);
}

double OnlyImuOdomNode::vectorNorm(const tf2::Vector3 & v)
{
  return std::sqrt(v.x() * v.x() + v.y() * v.y() + v.z() * v.z());
}

tf2::Vector3 OnlyImuOdomNode::applyDeadband(const tf2::Vector3 & v, double threshold)
{
  tf2::Vector3 out = v;
  if (std::abs(out.x()) < threshold) out.setX(0.0);
  if (std::abs(out.y()) < threshold) out.setY(0.0);
  if (std::abs(out.z()) < threshold) out.setZ(0.0);
  return out;
}

void OnlyImuOdomNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  tf2::Quaternion q;
  tf2::fromMsg(msg->orientation, q);

  const double q_norm2 = q.length2();
  if (!std::isfinite(q_norm2) || q_norm2 < 1.0e-12) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "Invalid IMU orientation quaternion. Waiting for a valid orientation.");
    return;
  }
  q.normalize();

  const tf2::Vector3 accel_body(
    msg->linear_acceleration.x,
    msg->linear_acceleration.y,
    msg->linear_acceleration.z);

  // The IMU measures gravity while stationary. Estimate accelerometer bias
  // in the IMU/body frame during an initial stationary calibration period.
  // q is treated as body -> odom/world rotation.
  const tf2::Vector3 gravity_world(0.0, 0.0, gravity_magnitude_);
  const tf2::Vector3 gravity_body = tf2::quatRotate(q.inverse(), gravity_world);

  if (!calibrated_) {
    accel_bias_sum_body_ += (accel_body - gravity_body);
    ++calibration_count_;

    if (calibration_count_ >= calibration_samples_) {
      accel_bias_body_ = accel_bias_sum_body_;
      accel_bias_body_ /= static_cast<double>(calibration_count_);
      calibrated_ = true;
      have_prev_stamp_ = false;
      have_prev_accel_ = false;
      have_filtered_accel_ = false;
      stationary_time_ = 0.0;

      RCLCPP_INFO(
        this->get_logger(),
        "Accelerometer calibration complete. bias_body = [%.6f, %.6f, %.6f] m/s^2",
        accel_bias_body_.x(), accel_bias_body_.y(), accel_bias_body_.z());
    }
    return;
  }

  rclcpp::Time stamp = msg->header.stamp;
  if (!have_prev_stamp_) {
    prev_stamp_ = stamp;
    have_prev_stamp_ = true;
    return;
  }

  const double dt = (stamp - prev_stamp_).seconds();
  prev_stamp_ = stamp;

  if (!std::isfinite(dt) || dt <= 0.0 || dt > max_dt_) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "Invalid IMU dt=%.6f s. Integration skipped.", dt);
    have_prev_accel_ = false;
    return;
  }

  // 1) Remove the initial accelerometer bias in the IMU/body frame.
  // 2) Rotate acceleration into odom/world frame.
  // 3) Remove gravity.
  const tf2::Vector3 accel_body_corrected = accel_body - accel_bias_body_;
  tf2::Vector3 acceleration_world =
    tf2::quatRotate(q, accel_body_corrected) - gravity_world;

  if (planar_mode_) {
    acceleration_world.setZ(0.0);
  }

  // First-order low-pass filter. alpha=1.0 disables filtering.
  if (!have_filtered_accel_) {
    filtered_accel_world_ = acceleration_world;
    have_filtered_accel_ = true;
  } else {
    filtered_accel_world_ =
      acceleration_world * accel_lpf_alpha_ +
      filtered_accel_world_ * (1.0 - accel_lpf_alpha_);
  }

  acceleration_world = applyDeadband(filtered_accel_world_, accel_deadband_);

  const tf2::Vector3 gyro_body(
    msg->angular_velocity.x,
    msg->angular_velocity.y,
    msg->angular_velocity.z);

  const bool stationary =
    vectorNorm(acceleration_world) <= stationary_accel_threshold_ &&
    vectorNorm(gyro_body) <= stationary_gyro_threshold_;

  if (stationary) {
    stationary_time_ += dt;
  } else {
    stationary_time_ = 0.0;
  }

  const tf2::Vector3 velocity_before = velocity_world_;

  // Trapezoidal integration: acceleration -> velocity.
  if (have_prev_accel_) {
    velocity_world_ += (prev_accel_world_ + acceleration_world) * (0.5 * dt);
  } else {
    velocity_world_ += acceleration_world * dt;
    have_prev_accel_ = true;
  }

  // Optional Zero Velocity Update (ZUPT). This is disabled by default because
  // an IMU alone cannot distinguish rest from straight constant-velocity motion.
  // Enable only for experiments where quasi-static intervals are known to be stops.
  if (zupt_enabled_ && stationary_time_ >= stationary_time_threshold_) {
    velocity_world_.setValue(0.0, 0.0, 0.0);
  }

  if (planar_mode_) {
    velocity_world_.setZ(0.0);
  }

  // Optional differential-drive nonholonomic constraint: remove lateral
  // velocity in the robot frame, then rotate the constrained velocity back.
  if (nonholonomic_constraint_) {
    tf2::Vector3 velocity_body = tf2::quatRotate(q.inverse(), velocity_world_);
    velocity_body.setY(0.0);
    if (planar_mode_) velocity_body.setZ(0.0);
    velocity_world_ = tf2::quatRotate(q, velocity_body);
  }

  // Trapezoidal integration: velocity -> position.
  position_world_ += (velocity_before + velocity_world_) * (0.5 * dt);
  if (planar_mode_) {
    position_world_.setZ(0.0);
  }

  prev_accel_world_ = acceleration_world;

  publishOdometry(msg, q, acceleration_world);
}

void OnlyImuOdomNode::publishOdometry(
  const sensor_msgs::msg::Imu::SharedPtr msg,
  const tf2::Quaternion & orientation,
  const tf2::Vector3 & acceleration_world)
{
  nav_msgs::msg::Odometry odom;
  odom.header.stamp = msg->header.stamp;
  odom.header.frame_id = odom_header_frame_;
  odom.child_frame_id = odom_child_frame_;

  odom.pose.pose.position.x = position_world_.x();
  odom.pose.pose.position.y = position_world_.y();
  odom.pose.pose.position.z = planar_mode_ ? 0.0 : position_world_.z();
  odom.pose.pose.orientation = tf2::toMsg(orientation);

  // nav_msgs/Odometry defines twist in child_frame_id. The integrated
  // velocity is stored in odom/world coordinates, so rotate it back.
  tf2::Vector3 velocity_body = tf2::quatRotate(orientation.inverse(), velocity_world_);
  if (planar_mode_) velocity_body.setZ(0.0);

  odom.twist.twist.linear.x = velocity_body.x();
  odom.twist.twist.linear.y = velocity_body.y();
  odom.twist.twist.linear.z = velocity_body.z();
  odom.twist.twist.angular = msg->angular_velocity;

  odom_pub_->publish(odom);

  if (publish_tf_ && tf_broadcaster_) {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = msg->header.stamp;
    transform.header.frame_id = tf_header_frame_;
    transform.child_frame_id = tf_child_frame_;
    transform.transform.translation.x = position_world_.x();
    transform.transform.translation.y = position_world_.y();
    transform.transform.translation.z = planar_mode_ ? 0.0 : position_world_.z();
    transform.transform.rotation = tf2::toMsg(orientation);
    tf_broadcaster_->sendTransform(transform);
  }

  if (debug_) {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "pos=[%.3f %.3f %.3f] vel_world=[%.3f %.3f %.3f] acc_world=[%.3f %.3f %.3f] stationary=%.3f s",
      position_world_.x(), position_world_.y(), position_world_.z(),
      velocity_world_.x(), velocity_world_.y(), velocity_world_.z(),
      acceleration_world.x(), acceleration_world.y(), acceleration_world.z(),
      stationary_time_);
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OnlyImuOdomNode>());
  rclcpp::shutdown();
  return 0;
}
