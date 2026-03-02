#pragma once

// Here we include message types which we can subscribe to or publish
#include <std_msgs/msg/float32.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <raptor_dbw_msgs/msg/steering_extended_report.hpp>
#include <raptor_dbw_msgs/msg/wheel_speed_report.hpp>
#include <novatel_oem7_msgs/msg/rawimu.hpp>
#include <vectornav_msgs/msg/common_group.hpp>

#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/rclcpp.hpp>
#include <cstdint>
#include <deque>

class TakeHome : public rclcpp::Node {
 public:
  TakeHome(const rclcpp::NodeOptions& options);

  void odometry_callback(nav_msgs::msg::Odometry::ConstSharedPtr odom_msg);

  void wheel_speed_callback(raptor_dbw_msgs::msg::WheelSpeedReport::ConstSharedPtr w_speed_msg);

  void steering_callback(raptor_dbw_msgs::msg::SteeringExtendedReport::ConstSharedPtr steering_msg);

  void get_wheel_slip();

  void top_imu_callback(novatel_oem7_msgs::msg::RAWIMU::ConstSharedPtr top_imu_msg);

  void bottom_imu_callback(novatel_oem7_msgs::msg::RAWIMU::ConstSharedPtr bottom_imu_msg);

  void vectornav_imu_callback(vectornav_msgs::msg::CommonGroup::ConstSharedPtr vectornav_imu_msg);

  double calculate_jitter(std::deque<int64_t>& imu_window);

  void curvilinear_distance_callback(std_msgs::msg::Float32::ConstSharedPtr curvilinear_msg);

 private:
  constexpr static float F_TRACK_WIDTH = 1.638; //front track width
  constexpr static float R_TRACK_WIDTH = 1.523; //rear track width
  constexpr static float COG_TO_FRONT_W = 1.7238; // cog to front wheels

  int64_t last_odometer_time_ = 0; // last time of topic data in nanoseconds
  double l_longitudinal_speed_; // linear longitudinal speed
  double yaw_rate_; // yaw rate
  double l_lateral_speed_; // linear lateral speed
  int64_t last_steering_time_ = 0;
  float wheel_angle_; 
  //Wheel speeds
  int64_t last_wheel_speed_time_ = 0; 
  double fl_speed_; //front left speed
  double fr_speed_; //front right speed
  double rl_speed_; //rear left speed
  double rr_speed_; //rear right speed

  //IMU timestamp holders for jitter
  std::deque<int64_t> top_imu_;
  std::deque<int64_t> bottom_imu_;
  std::deque<int64_t> vectornav_imu_;

  //start time of lap
  int64_t start_time_ = 0;
  double lap_distance_ = 0;
  // Subscribers and Publishers
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscriber_;
  rclcpp::Subscription<raptor_dbw_msgs::msg::WheelSpeedReport>::SharedPtr wheel_speed_subscriber_;
  rclcpp::Subscription<raptor_dbw_msgs::msg::SteeringExtendedReport>::SharedPtr steering_subscriber_;
  rclcpp::Subscription<novatel_oem7_msgs::msg::RAWIMU>::SharedPtr top_imu_subscriber_;
  rclcpp::Subscription<novatel_oem7_msgs::msg::RAWIMU>::SharedPtr bottom_imu_subscriber_;
  rclcpp::Subscription<vectornav_msgs::msg::CommonGroup>::SharedPtr vectornav_imu_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr curvilinear_subscriber_;

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr rr_slip_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr rl_slip_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr fr_slip_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr fl_slip_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr top_imu_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr bottom_imu_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr vectornav_imu_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr curvilinear_publisher_;
};
