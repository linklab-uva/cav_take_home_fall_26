#pragma once

// Here we include message types which we can subscribe to or publish
#include <std_msgs/msg/float32.hpp>
#include <nav_msgs/msg/odometry.hpp>

// wheel slip
#include <raptor_dbw_msgs/msg/steering_extended_report.hpp>
#include <raptor_dbw_msgs/msg/wheel_speed_report.hpp>

// imu
#include <novatel_oem7_msgs/msg/rawimu.hpp>
#include <vectornav_msgs/msg/common_group.hpp>

#include <cmath>
#include <numbers>
#include <deque>

#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/rclcpp.hpp>

class TakeHome : public rclcpp::Node {
 public:
  TakeHome(const rclcpp::NodeOptions& options);

  // wheel slip
  void odometry_callback(nav_msgs::msg::Odometry::ConstSharedPtr odom_msg);
  void wheel_speed_callback(raptor_dbw_msgs::msg::WheelSpeedReport::ConstSharedPtr wheel_speed_msg);
  void steering_callback(raptor_dbw_msgs::msg::SteeringExtendedReport::ConstSharedPtr steering_msg);
  void publish_wheel_slip();

  // imu
  void top_imu_callback(novatel_oem7_msgs::msg::RAWIMU::ConstSharedPtr top_imu_msg);
  void bottom_imu_callback(novatel_oem7_msgs::msg::RAWIMU::ConstSharedPtr bottom_imu_msg);
  void vectornav_imu_callback(vectornav_msgs::msg::CommonGroup::ConstSharedPtr vectornav_imu_msg);
  double calculate_jitter(std::deque<uint64_t> &timestamps, uint64_t new_timestamp);
  // void publish_imu_jitter(); // no need to collate results since imu jitters are independent

  // lap time
  void curvilinear_distance_callback(std_msgs::msg::Float32::ConstSharedPtr curvilinear_distance_msg);

 private:

  // Wheel slip --------------------------------------------------------------------------------------------
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscriber_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr metric_publisher_;

  // subscriber wheel speed report, steering angle data
  rclcpp::Subscription<raptor_dbw_msgs::msg::WheelSpeedReport>::SharedPtr wheel_speed_subscriber_;
  rclcpp::Subscription<raptor_dbw_msgs::msg::SteeringExtendedReport>::SharedPtr steering_subscriber_;

  // new publishers for each respective wheel slip topic
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr slip_long_rr_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr slip_long_rl_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr slip_long_fr_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr slip_long_fl_publisher_;

  bool received_odometry_{false};
  double vx_{0.0}, vy_{0.0};
  double angular_velocity_{0.0};

  bool received_wheel_speed_{false};
  double wheel_speed_rl_{0.0};
  double wheel_speed_rr_{0.0};
  double wheel_speed_fl_{0.0};
  double wheel_speed_fr_{0.0};

  bool received_steering_data_{false};
  double primary_steering_angle_fbk_{0.0};

  static constexpr double front_track_width_{1.638};
  static constexpr double rear_track_width_{1.523};
  static constexpr double dist_COG_front_{1.7238};

  // IMU ---------------------------------------------------------------------------------------------------

  rclcpp::Subscription<novatel_oem7_msgs::msg::RAWIMU>::SharedPtr top_imu_subscriber_;
  rclcpp::Subscription<novatel_oem7_msgs::msg::RAWIMU>::SharedPtr bottom_imu_subscriber_;
  rclcpp::Subscription<vectornav_msgs::msg::CommonGroup>::SharedPtr vectornav_imu_subscriber_;

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr top_imu_jitter_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr bottom_imu_jitter_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr vectornav_imu_jitter_publisher_;

  std::deque<uint64_t> top_imu_timestamps_;
  std::deque<uint64_t> bottom_imu_timestamps_;
  std::deque<uint64_t> vectornav_imu_timestamps_;

  // lap time ----------------------------------------------------------------------------------------------

  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr curvilinear_distance_subscriber_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr lap_time_publisher_;

  uint64_t lap_time_start_{0};
  double last_recorded_curvilinear_distance_{0.0};
  double track_length_{0.0};
};
