#pragma once

// Here we include message types which we can subscribe to or publish
#include <std_msgs/msg/float32.hpp>
#include <nav_msgs/msg/odometry.hpp> 

#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/rclcpp.hpp>

class TakeHome : public rclcpp::Node {
 public:
  TakeHome(const rclcpp::NodeOptions& options);
  // added line
  void curvilinear_distance_callback(std_msgs::msg::Float32::ConstSharedPtr msg);

  void odometry_callback(nav_msgs::msg::Odometry::ConstSharedPtr odom_msg);

 private:

  // Subscribers and Publishers
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscriber_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr metric_publisher_;

// Lap time tracking -  added line
rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr curvilinear_subscriber_;
rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr lap_time_publisher_;
double last_curvilinear_distance_ = 0.0;
rclcpp::Time lap_start_time_;
bool lap_in_progress_ = false;
};
