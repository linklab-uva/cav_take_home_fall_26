#include "take_home_node/take_home.hpp"
#include <rclcpp_components/register_node_macro.hpp>
#include <cmath>

TakeHome::TakeHome(const rclcpp::NodeOptions& options)
    : Node("take_home_metrics", options) {

    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    
    /* Part A */
    odometry_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "vehicle/uva_odometry", qos_profile,
      std::bind(&TakeHome::odometry_callback, this, std::placeholders::_1));

      // Publishers for metric and slip values
      metric_publisher_ = this->create_publisher<std_msgs::msg::Float32>("metrics_output", qos_profile);
      slip_rr_publisher_ = this->create_publisher<std_msgs::msg::Float32>("slip/long/rr", qos_profile);
      slip_rl_publisher_ = this->create_publisher<std_msgs::msg::Float32>("slip/long/rl", qos_profile);
      slip_fr_publisher_ = this->create_publisher<std_msgs::msg::Float32>("slip/long/fr", qos_profile);
      slip_fl_publisher_ = this->create_publisher<std_msgs::msg::Float32>("slip/long/fl", qos_profile);

    wheel_speed_subscriber_ = this->create_subscription<raptor_dbw_msgs::msg::WheelSpeedReport>(
      "raptor_dbw_interface/wheel_speed_report", qos_profile,
      std::bind(&TakeHome::wheel_speed_callback, this, std::placeholders::_1));

    steering_extended_subscriber_ = this->create_subscription<raptor_dbw_msgs::msg::SteeringExtendedReport>(
      "raptor_dbw_interface/steering_extended_report", qos_profile,
      std::bind(&TakeHome::steering_extended_callback, this, std::placeholders::_1));

    /* Part B */
    imu_top_subscriber_ = this->create_subscription<novatel_oem7_msgs::msg::RAWIMU>(
      "novatel_top/rawimu", qos_profile,
      std::bind(&TakeHome::imu_top_callback, this, std::placeholders::_1));

      imu_top_jitter_publisher_ = this->create_publisher<std_msgs::msg::Float32>("imu_top/jitter", qos_profile);

    imu_bottom_subscriber_ = this->create_subscription<novatel_oem7_msgs::msg::RAWIMU>(
      "novatel_bottom/rawimu", qos_profile,
      std::bind(&TakeHome::imu_bottom_callback, this, std::placeholders::_1));

      imu_bottom_jitter_publisher_ = this->create_publisher<std_msgs::msg::Float32>("imu_bottom/jitter", qos_profile);

    imu_vectornav_subscriber_ = this->create_subscription<vectornav_msgs::msg::CommonGroup>(
      "vectornav/raw/common", qos_profile,
      std::bind(&TakeHome::imu_vectornav_callback, this, std::placeholders::_1));

      imu_vectornav_jitter_publisher_ = this->create_publisher<std_msgs::msg::Float32>("imu_vectornav/jitter", qos_profile);
    
    /* Part C */
    curvilinear_distance_subscriber_ = this->create_subscription<std_msgs::msg::Float32>(
      "curvilinear_distance", qos_profile,
      std::bind(&TakeHome::curvilinear_distance_callback, this, std::placeholders::_1));

      lap_time_publisher_ = this->create_publisher<std_msgs::msg::Float32>("lap_time", qos_profile);
}

// 
/**
 * Part A: Wheel Spin Ratio
 * 
 * Whenever a message is published to the topic "vehicle/uva_odometry" the subscriber passes the message onto this callback
 * To see what is in each message look for the corresponding .msg file in this repository
 * For instance, when running `ros2 bag info` on the given bag, we see the wheel speed report has message type of raptor_dbw_msgs/msgs/WheelSpeedReport
 * and from the corresponding WheelSpeedReport.msg file we see that we can do msg->front_left for the front left speed for instance.
 * For the built in ROS2 messages, we can find the documentation online: e.g. https://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html
 */
void TakeHome::odometry_callback(nav_msgs::msg::Odometry::ConstSharedPtr odom_msg) {
  // Original dummy metric
  float position_x = odom_msg->pose.pose.position.x;
  float position_y = odom_msg->pose.pose.position.y;
  float position_z = odom_msg->pose.pose.position.z;

  std_msgs::msg::Float32 metric_msg;
  metric_msg.data = (position_x + position_y + position_z) / (position_x + position_z); // Example metric calculation
  metric_publisher_->publish(metric_msg);

  // No wheel speed or steering data -> wait
  if (!latest_wheel_speeds_ || !latest_steering_) {
      return;
  }

  // Car's linear and angular velocity from odometry
  float v_x = odom_msg->twist.twist.linear.x; // in m/s
  float v_y = odom_msg->twist.twist.linear.y;
  float omega = odom_msg->twist.twist.angular.z;  // in rad/s

  // Wheel speeds from wheel speed report (kmph -> m/s)
  float v_w_rr = latest_wheel_speeds_->rear_right / 3.6f;
  float v_w_rl = latest_wheel_speeds_->rear_left / 3.6f;
  float v_w_fr = latest_wheel_speeds_->front_right / 3.6f;
  float v_w_fl = latest_wheel_speeds_->front_left / 3.6f;

  // Wheel angle (deg / 15 -> rad)
  float delta = latest_steering_->primary_steering_angle_fbk / STEERING_RATIO * M_PI / 180.0f;

  // Calculate slip ratios for rear wheels w/ zero-division handling
  float v_x_rr = v_x - 0.5f * omega * W_R;
  float v_x_rl = v_x + 0.5f * omega * W_R;

  float k_rr = (std::abs(v_x_rr) > EPSILON) ? (v_w_rr - v_x_rr) / v_x_rr : 0.0f;
  float k_rl = (std::abs(v_x_rl) > EPSILON) ? (v_w_rl - v_x_rl) / v_x_rl : 0.0f;

  // Calculate slip ratios for front wheels w/ zero-division handling
  float v_x_fr = v_x - 0.5f * omega * W_F;
  float v_x_fl = v_x + 0.5f * omega * W_F;

  float v_y_fr = v_y + omega * L_F;
  float v_y_fl = v_y + omega * L_F;

  float v_delta_x_fr = std::cos(delta) * v_x_fr - std::sin(delta) * v_y_fr;
  float v_delta_x_fl = std::cos(delta) * v_x_fl - std::sin(delta) * v_y_fl;

  float k_fr = (std::abs(v_delta_x_fr) > EPSILON) ? (v_w_fr - v_delta_x_fr) / v_delta_x_fr : 0.0f;
  float k_fl = (std::abs(v_delta_x_fl) > EPSILON) ? (v_w_fl - v_delta_x_fl) / v_delta_x_fl : 0.0f;

  // Publish slip ratios
  std_msgs::msg::Float32 slip_rr_msg;
  slip_rr_msg.data = k_rr;
  slip_rr_publisher_->publish(slip_rr_msg);

  std_msgs::msg::Float32 slip_rl_msg;
  slip_rl_msg.data = k_rl;
  slip_rl_publisher_->publish(slip_rl_msg);

  std_msgs::msg::Float32 slip_fr_msg;
  slip_fr_msg.data = k_fr;
  slip_fr_publisher_->publish(slip_fr_msg);

  std_msgs::msg::Float32 slip_fl_msg;
  slip_fl_msg.data = k_fl;
  slip_fl_publisher_->publish(slip_fl_msg);
}

void TakeHome::wheel_speed_callback(raptor_dbw_msgs::msg::WheelSpeedReport::ConstSharedPtr wheel_speed_msg) {
  latest_wheel_speeds_ = wheel_speed_msg;
}

void TakeHome::steering_extended_callback(raptor_dbw_msgs::msg::SteeringExtendedReport::ConstSharedPtr steering_extended_msg) {
  latest_steering_ = steering_extended_msg;
}

/**
 * Part B: Jitter
 * 
 * Shared helper for calculating/publishing jitter for all 3 IMU topics
 *  */
void TakeHome::calculate_and_publish_jitter(
  JitterStruct& state,
  const rclcpp::Time& stamp,
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr& pub) {

  // If first msg, no delta_t to calculate -> only initialize state
  if (state.is_first_msg) {
    state.prev_stamp = stamp;
    state.is_first_msg = false;
    return;
  }

  // If not first msg, calculate delta_t and jitter
  double delta_t = (stamp - state.prev_stamp).seconds();  // double for precision for time
  state.prev_stamp = stamp;

  // Add new delta_t to sliding window and remove old ones outside of 1 second
  double cur_time = stamp.seconds();
  state.window.push_back({cur_time, delta_t});

  while (!state.window.empty() && state.window.front().first < cur_time - 1.0) {
    state.window.pop_front();
  }

  // Calculate jitter = variance(delta_t) in the current window
  if (state.window.size() > 1) {  // need at least 2 samples for sample variance
    double mean = 0.0;
    for (const auto& entry : state.window) {
      mean += entry.second;
    }
    mean /= state.window.size();

    double variance = 0.0;
    for (const auto& entry : state.window) {
      variance += std::pow(entry.second - mean, 2);
    }
    variance /= (state.window.size() - 1); // Sample variance

    // Publish jitter
    std_msgs::msg::Float32 jitter_msg;
    jitter_msg.data = static_cast<float>(variance); // convert back to intended type
    pub->publish(jitter_msg);
  }
}

void TakeHome::imu_top_callback(novatel_oem7_msgs::msg::RAWIMU::ConstSharedPtr msg) {
  calculate_and_publish_jitter(imu_top_state_, rclcpp::Time(msg->header.stamp), imu_top_jitter_publisher_);
}

void TakeHome::imu_bottom_callback(novatel_oem7_msgs::msg::RAWIMU::ConstSharedPtr msg) {
  calculate_and_publish_jitter(imu_bottom_state_, rclcpp::Time(msg->header.stamp), imu_bottom_jitter_publisher_);
}

void TakeHome::imu_vectornav_callback(vectornav_msgs::msg::CommonGroup::ConstSharedPtr msg) {
  calculate_and_publish_jitter(imu_vectornav_state_, rclcpp::Time(msg->header.stamp), imu_vectornav_jitter_publisher_);
}

/**
 * Part C: Lap Time
 *
 * Idea: Curvilinear distance reset to zero at start of each lap
 * - Lap completion = significant enough drop in curvilinear distance
 */
void TakeHome::curvilinear_distance_callback(std_msgs::msg::Float32::ConstSharedPtr msg) {
    float cur_distance = msg->data;
    double cur_time = this->now().seconds();

    // If first msg, only initialize state
    if (is_first_msg_) {
        prev_curvilinear_distance_ = cur_distance;
        lap_start_time_ = cur_time;
        is_first_msg_ = false;
        return;
    }

    // Detect lap completion: distance suddenly drops by 10%+
    if (cur_distance < prev_curvilinear_distance_ * 0.9f) {
        double lap_time = cur_time - lap_start_time_;
        lap_start_time_ = cur_time;

        std_msgs::msg::Float32 lap_time_msg;
        lap_time_msg.data = static_cast<float>(lap_time);
        lap_time_publisher_->publish(lap_time_msg);
    }

    prev_curvilinear_distance_ = cur_distance;
}


RCLCPP_COMPONENTS_REGISTER_NODE(TakeHome)
