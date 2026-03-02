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
#include <std_msgs/msg/float32.hpp>

class TakeHome : public rclcpp::Node {
 public:
  TakeHome(const rclcpp::NodeOptions& options);

  void odometry_callback(nav_msgs::msg::Odometry::ConstSharedPtr odom_msg);
  void wheel_speed_callback(raptor_dbw_msgs::msg::WheelSpeedReport::ConstSharedPtr msg);
  void steering_callback(raptor_dbw_msgs::msg::SteeringExtendedReport::ConstSharedPtr msg);

  void imu_top_callback(novatel_oem7_msgs::msg::RAWIMU::ConstSharedPtr msg);

  void imu_bottom_callback(novatel_oem7_msgs::msg::RAWIMU::ConstSharedPtr msg);

  void imu_vectornav_callback(vectornav_msgs::msg::CommonGroup::ConstSharedPtr msg);

  void curvilinear_callback(std_msgs::msg::Float32::ConstSharedPtr msg);

  float compute_jitter(std::deque<double>& times, double current_time);
 private:

  // Subscribers and Publishers
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscriber_;
  rclcpp::Subscription<raptor_dbw_msgs::msg::WheelSpeedReport>::SharedPtr wheel_speed_subscriber_;
  rclcpp::Subscription<raptor_dbw_msgs::msg::SteeringExtendedReport>::SharedPtr steering_subscriber_;


  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr metric_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr slip_rr_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr slip_rl_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr slip_fr_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr slip_fl_publisher_;

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr imu_top_jitter_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr imu_bottom_jitter_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr imu_vectornav_jitter_pub_;


  //part b

  rclcpp::Subscription<novatel_oem7_msgs::msg::RAWIMU>::SharedPtr imu_top_subscriber_;
  rclcpp::Subscription<novatel_oem7_msgs::msg::RAWIMU>::SharedPtr imu_bottom_subscriber_;
  rclcpp::Subscription<vectornav_msgs::msg::CommonGroup>::SharedPtr imu_vectornav_subscriber_;

  //part c

  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr curvilinear_subscriber_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr lap_time_publisher_;


  //inittialize the variables we get from the topics
  float rear_right_ = 0.0f;
  float rear_left_ = 0.0f;
  float front_right_ = 0.0f;
  float front_left_ = 0.0f;
  float steering_motor_angle_deg_ = 0.0f;
  

  std::deque<double> imu_top_times_;
  std::deque<double> imu_bottom_times_;
  std::deque<double> imu_vectornav_times_;

  float previous_distance_ = 0.0f;
  double last_lap_time_ = 0.0;
  bool first_distance_received_ = false;
};
