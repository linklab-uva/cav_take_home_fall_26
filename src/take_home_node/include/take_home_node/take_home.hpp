#pragma once

// Here we include message types which we can subscribe to or publish
#include <std_msgs/msg/float32.hpp>
#include <nav_msgs/msg/odometry.hpp> 
#include <raptor_dbw_msgs/msg/wheel_speed_report.hpp>
#include <raptor_dbw_msgs/msg/steering_extended_report.hpp>
#include <novatel_oem7_msgs/msg/rawimu.hpp>
#include <vectornav_msgs/msg/common_group.hpp>


#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/rclcpp.hpp>
#include <deque>
#include <cmath>

class TakeHome : public rclcpp::Node {
 public:
  TakeHome(const rclcpp::NodeOptions& options);

  void odometry_callback(nav_msgs::msg::Odometry::ConstSharedPtr odom_msg);
  void wheel_speed_callback(raptor_dbw_msgs::msg::WheelSpeedReport::ConstSharedPtr wheel_speed_msg);
  void steering_callback(raptor_dbw_msgs::msg::SteeringExtendedReport::ConstSharedPtr steering_msg);
  void top_imu_callback(novatel_oem7_msgs::msg::RAWIMU::ConstSharedPtr top_imu_msg);
  void bottom_imu_callback(novatel_oem7_msgs::msg::RAWIMU::ConstSharedPtr bottom_imu_msg);
  void vector_nav_callback(vectornav_msgs::msg::CommonGroup::ConstSharedPtr vectornav_msg);
  void curvilinear_callback(std_msgs::msg::Float32::ConstSharedPtr curvilinear_msg);
  double calc_jitter(std::deque<uint64_t> &timestamps, uint64_t newTime);

 private:

  // Subscribers
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscriber_;
  rclcpp::Subscription<raptor_dbw_msgs::msg::WheelSpeedReport>::SharedPtr wheelSpeed_subscriber_;
  rclcpp::Subscription<raptor_dbw_msgs::msg::SteeringExtendedReport>::SharedPtr steering_subscriber_;
  rclcpp::Subscription<novatel_oem7_msgs::msg::RAWIMU>::SharedPtr topIMU_subscriber_;
  rclcpp::Subscription<novatel_oem7_msgs::msg::RAWIMU>::SharedPtr bottomIMU_subscriber_;
  rclcpp::Subscription<vectornav_msgs::msg::CommonGroup>::SharedPtr vectorNavIMU_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr curvilinear_subscriber_;

  // Publishers
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr metric_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr wheelSlip_rr_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr wheelSlip_rl_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr wheelSlip_fr_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr wheelSlip_fl_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr topIMU_jitter_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr bottomIMU_jitter_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr navIMU_jitter_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr laptime_publisher_;

  // Deques to store timestamps for jitter calculations
  std::deque<uint64_t> top_timestamps_;
  std::deque<uint64_t> bottom_timestamps_;
  std::deque<uint64_t> nav_timestamps_;

  // State variables for odometry/wheel slip calculations
  float vx = 0.0f;
  float vy = 0.0f;
  float w = 0.0f;
  float vx_rr = 0.0f;
  float vx_rl = 0.0f;
  float vx_fr = 0.0f;
  float vx_fl = 0.0f;
  float vy_fr = 0.0f;
  float vy_fl = 0.0f;
  float vx_delta_fr = 0.0f;
  float vx_delta_fl = 0.0f;
  float k_rr = 0.0f;
  float k_rl = 0.0f;
  float k_fr = 0.0f;
  float k_fl = 0.0f;

  // Latest wheel speeds
  float latest_wheel_speed_rr = 0.0f;
  float latest_wheel_speed_rl = 0.0f;
  float latest_wheel_speed_fr = 0.0f;
  float latest_wheel_speed_fl = 0.0f;

  // Latest steering angle
  float primary_steering_angle_fbk = 0.0f;

  // Curvilinear distance
  float last_curvilinear_distance = 0.0f;
  uint64_t last_lap_time = 0;
};
