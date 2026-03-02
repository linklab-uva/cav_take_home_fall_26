#pragma once

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

struct JitterWindow {
    std::deque<std::pair<double, double>> history;
    double last_time = -1.0;

    bool calculate_jitter(double current_time, float& out_jitter) {
        if (last_time < 0.0) { 
            last_time = current_time; 
            return false; 
        }

        double dt = current_time - last_time;
        history.push_back({current_time, dt});
        last_time = current_time;

        while (!history.empty() && (current_time - history.front().first) > 1.0) {
            history.pop_front();
        }

        if (history.size() < 2) return false;

        //mean
        double sum = 0.0;
        for (const auto& p : history) sum += p.second;
        double mean = sum / history.size();

        //variance
        double variance_sum = 0.0;
        for (const auto& p : history) {
            variance_sum += (p.second - mean) * (p.second - mean);
        }
        
        out_jitter = static_cast<float>(variance_sum / history.size());
        return true;
    }
};

class TakeHome : public rclcpp::Node {
 public:
  TakeHome(const rclcpp::NodeOptions& options);

 private:
  // Callbacks
  void odometry_callback(nav_msgs::msg::Odometry::ConstSharedPtr odom_msg);
  void wheel_speed_callback(raptor_dbw_msgs::msg::WheelSpeedReport::ConstSharedPtr ws_msg);
  void steering_callback(raptor_dbw_msgs::msg::SteeringExtendedReport::ConstSharedPtr steer_msg);

  void top_imu_callback(novatel_oem7_msgs::msg::RAWIMU::ConstSharedPtr msg);
  void bottom_imu_callback(novatel_oem7_msgs::msg::RAWIMU::ConstSharedPtr msg);
  void vectornav_callback(vectornav_msgs::msg::CommonGroup::ConstSharedPtr msg);

  void curvilinear_callback(std_msgs::msg::Float32::ConstSharedPtr msg);

  // Subscribers
  rclcpp::Subscription<novatel_oem7_msgs::msg::RAWIMU>::SharedPtr top_imu_subscriber_;
  rclcpp::Subscription<novatel_oem7_msgs::msg::RAWIMU>::SharedPtr bottom_imu_subscriber_;
  rclcpp::Subscription<vectornav_msgs::msg::CommonGroup>::SharedPtr vectornav_subscriber_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscriber_;
  rclcpp::Subscription<raptor_dbw_msgs::msg::WheelSpeedReport>::SharedPtr wheel_speed_subscriber_;
  rclcpp::Subscription<raptor_dbw_msgs::msg::SteeringExtendedReport>::SharedPtr steering_subscriber_;

  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr curvilinear_subscriber_;

  // Wheel slip metrics
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_slip_rr_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_slip_rl_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_slip_fr_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_slip_fl_;

  // Jitter 
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_jitter_top_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_jitter_bottom_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_jitter_vectornav_;

  // Lap time 
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_lap_time_;

  float wheel_speed_fl_ = 0.0;
  float wheel_speed_fr_ = 0.0;
  float wheel_speed_rl_ = 0.0;
  float wheel_speed_rr_ = 0.0;
  float steering_angle_rad_ = 0.0;

  // Constants
  const float w_f = 1.638;
  const float w_r = 1.523;
  const float l_f = 1.7238;

  JitterWindow top_imu_window_;
  JitterWindow bottom_imu_window_;
  JitterWindow vectornav_window_;

  double latest_odom_time_ = -1.0;
  double lap_start_time_ = -1.0;
  float last_s_ = -1.0;
};
