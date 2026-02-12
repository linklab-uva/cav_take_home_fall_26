#pragma once

// Here we include message types which we can subscribe to or publish
#include <deque>
#include <std_msgs/msg/float32.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <novatel_oem7_msgs/msg/rawimu.hpp>
#include <raptor_dbw_msgs/msg/steering_extended_report.hpp>
#include <raptor_dbw_msgs/msg/wheel_speed_report.hpp>
#include <vectornav_msgs/msg/common_group.hpp>

#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/rclcpp.hpp>

class TakeHome : public rclcpp::Node {
 public:
  TakeHome(const rclcpp::NodeOptions& options);

  void odometry_callback(nav_msgs::msg::Odometry::ConstSharedPtr odom_msg);
  void wheel_speed_callback(raptor_dbw_msgs::msg::WheelSpeedReport::ConstSharedPtr msg);
  void steering_callback(raptor_dbw_msgs::msg::SteeringExtendedReport::ConstSharedPtr msg);
  void imu_top_callback(novatel_oem7_msgs::msg::RAWIMU::ConstSharedPtr msg);
  void imu_bottom_callback(novatel_oem7_msgs::msg::RAWIMU::ConstSharedPtr msg);
  void imu_vectornav_callback(vectornav_msgs::msg::CommonGroup::ConstSharedPtr msg);
  void curvilinear_distance_callback(std_msgs::msg::Float32::ConstSharedPtr msg);

 private:

  void publish_slip_metrics();
  double update_jitter(std::deque<double>& timestamps_sec, const rclcpp::Time& stamp);

  // Subscribers and Publishers
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscriber_;
  rclcpp::Subscription<raptor_dbw_msgs::msg::WheelSpeedReport>::SharedPtr wheel_speed_subscriber_;
  rclcpp::Subscription<raptor_dbw_msgs::msg::SteeringExtendedReport>::SharedPtr steering_subscriber_;
  rclcpp::Subscription<novatel_oem7_msgs::msg::RAWIMU>::SharedPtr imu_top_subscriber_;
  rclcpp::Subscription<novatel_oem7_msgs::msg::RAWIMU>::SharedPtr imu_bottom_subscriber_;
  rclcpp::Subscription<vectornav_msgs::msg::CommonGroup>::SharedPtr imu_vectornav_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr curvilinear_distance_subscriber_;

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr slip_rr_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr slip_rl_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr slip_fl_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr slip_fr_publisher_;

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr imu_top_jitter_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr imu_bottom_jitter_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr imu_vectornav_jitter_publisher_;

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr lap_time_publisher_;

  bool odom_received_{false};
  bool wheel_speed_received_{false};
  bool steering_received_{false};

  double vx_{0.0};
  double vy_{0.0};
  double yaw_rate_{0.0};

  double wheel_fl_{0.0};
  double wheel_fr_{0.0};
  double wheel_rl_{0.0};
  double wheel_rr_{0.0};

  double steering_angle_rad_{0.0};

  std::deque<double> imu_top_timestamps_;
  std::deque<double> imu_bottom_timestamps_;
  std::deque<double> imu_vectornav_timestamps_;

  bool lap_time_initialized_{false};
  double last_curvilinear_distance_{0.0};
  rclcpp::Time last_lap_start_time_;

};
