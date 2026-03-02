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
#include <optional>

class TakeHome : public rclcpp::Node {
 public:
  TakeHome(const rclcpp::NodeOptions& options);

  void odometry_callback(nav_msgs::msg::Odometry::ConstSharedPtr odom_msg);
  void wheel_speed_callback(raptor_dbw_msgs::msg::WheelSpeedReport::ConstSharedPtr msg);
  void steering_callback(raptor_dbw_msgs::msg::SteeringExtendedReport::ConstSharedPtr msg);
  void curvilinear_distance_callback(std_msgs::msg::Float32::ConstSharedPtr msg);
  void imu_top_callback(novatel_oem7_msgs::msg::RAWIMU::ConstSharedPtr msg);
  void imu_bottom_callback(novatel_oem7_msgs::msg::RAWIMU::ConstSharedPtr msg);
  void imu_vectornav_callback(vectornav_msgs::msg::CommonGroup::ConstSharedPtr msg);
  void publish_metrics_timer_callback();

 private:
  // Helper functions
  void add_imu_timestamp(std::deque<std::pair<double, double>>& timestamp_deque, 
                         const std_msgs::msg::Header& header);
  double compute_imu_jitter(const std::deque<std::pair<double, double>>& timestamp_deque);
  std::optional<std::tuple<double, double, double, double>> compute_wheel_slip_ratios();

  // Subscribers
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscriber_;
  rclcpp::Subscription<raptor_dbw_msgs::msg::WheelSpeedReport>::SharedPtr wheel_speed_subscriber_;
  rclcpp::Subscription<raptor_dbw_msgs::msg::SteeringExtendedReport>::SharedPtr steering_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr curvilinear_distance_subscriber_;
  rclcpp::Subscription<novatel_oem7_msgs::msg::RAWIMU>::SharedPtr imu_top_subscriber_;
  rclcpp::Subscription<novatel_oem7_msgs::msg::RAWIMU>::SharedPtr imu_bottom_subscriber_;
  rclcpp::Subscription<vectornav_msgs::msg::CommonGroup>::SharedPtr imu_vectornav_subscriber_;

  // Publishers
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr metric_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr slip_rr_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr slip_rl_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr slip_fr_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr slip_fl_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr imu_top_jitter_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr imu_bottom_jitter_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr imu_vectornav_jitter_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr lap_time_publisher_;

  // Timer for periodic metric publishing
  rclcpp::TimerBase::SharedPtr metrics_timer_;

  // State variables
  nav_msgs::msg::Odometry::ConstSharedPtr latest_odom_;
  raptor_dbw_msgs::msg::WheelSpeedReport::ConstSharedPtr latest_wheel_speed_;
  raptor_dbw_msgs::msg::SteeringExtendedReport::ConstSharedPtr latest_steering_;
  std::optional<double> latest_curvilinear_distance_;
  std::optional<double> prev_curvilinear_distance_;

  // IMU timestamp tracking (sliding windows for last 1 second)
  std::deque<std::pair<double, double>> imu_top_timestamps_;  // (node_time, msg_time)
  std::deque<std::pair<double, double>> imu_bottom_timestamps_;
  std::deque<std::pair<double, double>> imu_vectornav_timestamps_;

  // Lap time tracking
  std::optional<double> lap_start_time_;
  double lap_time_;
  bool is_new_lap_;

  // Constants
  static constexpr double WF = 1.638;   // Front track width (m)
  static constexpr double WR = 1.523;   // Rear track width (m)
  static constexpr double LF = 1.7238;  // CG to front axle distance (m)
  static constexpr double STEERING_RATIO = 15.0;
  static constexpr double EPSILON = 1e-3;  // Threshold for division-by-zero checks
};
