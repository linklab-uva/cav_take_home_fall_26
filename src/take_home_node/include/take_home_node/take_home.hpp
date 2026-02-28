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

// Needed to use queue for efficient sliding window filter
#include <deque>
#include <utility>

class TakeHome : public rclcpp::Node {
 public:
  TakeHome(const rclcpp::NodeOptions& options);

  // A: Called every time msg arrives on the odometry, wheel speed, steering report topics
  void odometry_callback(nav_msgs::msg::Odometry::ConstSharedPtr odom_msg);
  void wheel_speed_callback(raptor_dbw_msgs::msg::WheelSpeedReport::ConstSharedPtr wheel_speed_msg);
  void steering_extended_callback(raptor_dbw_msgs::msg::SteeringExtendedReport::ConstSharedPtr steering_extended_msg); 
  
  // B: Called every time IMU top/bottom and VectorNav topics receive msgs
  void imu_top_callback(novatel_oem7_msgs::msg::RAWIMU::ConstSharedPtr msg);
  void imu_bottom_callback(novatel_oem7_msgs::msg::RAWIMU::ConstSharedPtr msg);
  void imu_vectornav_callback(vectornav_msgs::msg::CommonGroup::ConstSharedPtr msg);

  // C: Called every time curvilinear distance topic receives a msg
  void curvilinear_distance_callback(std_msgs::msg::Float32::ConstSharedPtr msg);

 private:

  // Subscribers
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscriber_;
  rclcpp::Subscription<raptor_dbw_msgs::msg::WheelSpeedReport>::SharedPtr wheel_speed_subscriber_;
  rclcpp::Subscription<raptor_dbw_msgs::msg::SteeringExtendedReport>::SharedPtr steering_extended_subscriber_;

  rclcpp::Subscription<novatel_oem7_msgs::msg::RAWIMU>::SharedPtr imu_top_subscriber_;
  rclcpp::Subscription<novatel_oem7_msgs::msg::RAWIMU>::SharedPtr imu_bottom_subscriber_;
  rclcpp::Subscription<vectornav_msgs::msg::CommonGroup>::SharedPtr imu_vectornav_subscriber_;

  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr curvilinear_distance_subscriber_;

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

  /* Part A */
  // Latest messages -> used to store wheel speed & steering report msgs to use in odometry callback for metric calculation
  raptor_dbw_msgs::msg::WheelSpeedReport::ConstSharedPtr latest_wheel_speeds_;
  raptor_dbw_msgs::msg::SteeringExtendedReport::ConstSharedPtr latest_steering_;

  // Vehicle constants provided in instructions
  static constexpr float W_F = 1.638f;
  static constexpr float W_R = 1.523f;
  static constexpr float L_F = 1.7238f;
  static constexpr float STEERING_RATIO = 15.0f;
  static constexpr float EPSILON = 1e-5f; // Small constant to prevent division by zero

  /* Part B */
  // Struct helper for jitter calculation
  struct JitterStruct {
    bool is_first_msg = true;
    rclcpp::Time prev_stamp{0, 0, RCL_ROS_TIME};
    std::deque<std::pair<double, double>> window; // (timestamp, delta_t)
  };

  // Stores state for jitter calculation (between messages) for each IMU topic
  JitterStruct imu_top_state_;
  JitterStruct imu_bottom_state_;
  JitterStruct imu_vectornav_state_;

  void calculate_and_publish_jitter(
    JitterStruct& state,
    const rclcpp::Time& stamp,
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr& pub);
  
  /* Part C */
  bool is_first_msg_ = true;
  float prev_curvilinear_distance_ = 0.0f;  // if 0 again, lap has restarted
  double lap_start_time_ = 0.0;  // seconds (from node clock)
};
