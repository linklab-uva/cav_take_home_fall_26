#include "take_home_node/take_home.hpp"
#include <cmath>
#include <vector>
#include <rclcpp_components/register_node_macro.hpp>

TakeHome::TakeHome(const rclcpp::NodeOptions& options)
    : Node("take_home_metrics", options) {

    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    
    // Look at the hpp file to define all class variables, including subscribers
    // A subscriber will "listen" to a topic and whenever a message is published to it, the subscriber
    // will pass it onto the attached callback (`TakeHome::odometry_callback` in this instance)
    odometry_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "vehicle/uva_odometry", qos_profile,
      std::bind(&TakeHome::odometry_callback, this, std::placeholders::_1));

    wheel_speed_subscriber_ = this->create_subscription<raptor_dbw_msgs::msg::WheelSpeedReport>(
      "raptor_dbw_interface/wheel_speed_report", qos_profile,
      std::bind(&TakeHome::wheel_speed_callback, this, std::placeholders::_1));

    steering_subscriber_ = this->create_subscription<raptor_dbw_msgs::msg::SteeringExtendedReport>(
      "raptor_dbw_interface/steering_extended_report", qos_profile,
      std::bind(&TakeHome::steering_callback, this, std::placeholders::_1));

    imu_top_subscriber_ = this->create_subscription<novatel_oem7_msgs::msg::RAWIMU>(
      "novatel_top/rawimu", qos_profile,
      std::bind(&TakeHome::imu_top_callback, this, std::placeholders::_1));

    imu_bottom_subscriber_ = this->create_subscription<novatel_oem7_msgs::msg::RAWIMU>(
      "novatel_bottom/rawimu", qos_profile,
      std::bind(&TakeHome::imu_bottom_callback, this, std::placeholders::_1));

    imu_vectornav_subscriber_ = this->create_subscription<vectornav_msgs::msg::CommonGroup>(
      "vectornav/raw/common", qos_profile,
      std::bind(&TakeHome::imu_vectornav_callback, this, std::placeholders::_1));

    curvilinear_distance_subscriber_ = this->create_subscription<std_msgs::msg::Float32>(
      "curvilinear_distance", qos_profile,
      std::bind(&TakeHome::curvilinear_distance_callback, this, std::placeholders::_1));

    slip_rr_publisher_ = this->create_publisher<std_msgs::msg::Float32>("slip/long/rr", qos_profile);
    slip_rl_publisher_ = this->create_publisher<std_msgs::msg::Float32>("slip/long/rl", qos_profile);
    slip_fl_publisher_ = this->create_publisher<std_msgs::msg::Float32>("slip/long/fl", qos_profile);
    slip_fr_publisher_ = this->create_publisher<std_msgs::msg::Float32>("slip/long/fr", qos_profile);

    imu_top_jitter_publisher_ = this->create_publisher<std_msgs::msg::Float32>("imu_top/jitter", qos_profile);
    imu_bottom_jitter_publisher_ = this->create_publisher<std_msgs::msg::Float32>("imu_bottom/jitter", qos_profile);
    imu_vectornav_jitter_publisher_ = this->create_publisher<std_msgs::msg::Float32>("imu_vectornav/jitter", qos_profile);

    lap_time_publisher_ = this->create_publisher<std_msgs::msg::Float32>("lap_time", qos_profile);
}

// 
/**
 * Whenever a message is published to the topic "vehicle/uva_odometry" the subscriber passes the message onto this callback
 * To see what is in each message look for the corresponding .msg file in this repository
 * For instance, when running `ros2 bag info` on the given bag, we see the wheel speed report has message type of raptor_dbw_msgs/msgs/WheelSpeedReport
 * and from the corresponding WheelSpeedReport.msg file we see that we can do msg->front_left for the front left speed for instance.
 * For the built in ROS2 messages, we can find the documentation online: e.g. https://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html
 */
void TakeHome::odometry_callback(nav_msgs::msg::Odometry::ConstSharedPtr odom_msg) {
  vx_ = odom_msg->twist.twist.linear.x;
  vy_ = odom_msg->twist.twist.linear.y;
  yaw_rate_ = odom_msg->twist.twist.angular.z;
  odom_received_ = true;

  publish_slip_metrics();
}

void TakeHome::wheel_speed_callback(raptor_dbw_msgs::msg::WheelSpeedReport::ConstSharedPtr msg) {
  constexpr double kKphToMps = 1000.0 / 3600.0;
  wheel_fl_ = static_cast<double>(msg->front_left) * kKphToMps;
  wheel_fr_ = static_cast<double>(msg->front_right) * kKphToMps;
  wheel_rl_ = static_cast<double>(msg->rear_left) * kKphToMps;
  wheel_rr_ = static_cast<double>(msg->rear_right) * kKphToMps;
  wheel_speed_received_ = true;

  publish_slip_metrics();
}

void TakeHome::steering_callback(raptor_dbw_msgs::msg::SteeringExtendedReport::ConstSharedPtr msg) {
  constexpr double kSteeringRatio = 15.0;
  const double wheel_angle_deg = static_cast<double>(msg->primary_steering_angle_fbk) / kSteeringRatio;
  steering_angle_rad_ = wheel_angle_deg * M_PI / 180.0;
  steering_received_ = true;

  publish_slip_metrics();
}

double TakeHome::update_jitter(std::deque<double>& timestamps_sec, const rclcpp::Time& stamp) {
  constexpr double kWindowSeconds = 1.0;
  const double current_time = stamp.seconds();
  timestamps_sec.push_back(current_time);

  while (!timestamps_sec.empty() && (current_time - timestamps_sec.front()) > kWindowSeconds) {
    timestamps_sec.pop_front();
  }

  if (timestamps_sec.size() < 3) {
    return 0.0;
  }

  std::vector<double> deltas;
  deltas.reserve(timestamps_sec.size() - 1);
  for (size_t i = 1; i < timestamps_sec.size(); ++i) {
    deltas.push_back(timestamps_sec[i] - timestamps_sec[i - 1]);
  }

  double mean = 0.0;
  for (double dt : deltas) {
    mean += dt;
  }
  mean /= static_cast<double>(deltas.size());

  double variance = 0.0;
  for (double dt : deltas) {
    const double diff = dt - mean;
    variance += diff * diff;
  }
  variance /= static_cast<double>(deltas.size());

  return variance;
}

void TakeHome::imu_top_callback(novatel_oem7_msgs::msg::RAWIMU::ConstSharedPtr msg) {
  const double jitter = update_jitter(imu_top_timestamps_, rclcpp::Time(msg->header.stamp));
  std_msgs::msg::Float32 out;
  out.data = static_cast<float>(jitter);
  imu_top_jitter_publisher_->publish(out);
}

void TakeHome::imu_bottom_callback(novatel_oem7_msgs::msg::RAWIMU::ConstSharedPtr msg) {
  const double jitter = update_jitter(imu_bottom_timestamps_, rclcpp::Time(msg->header.stamp));
  std_msgs::msg::Float32 out;
  out.data = static_cast<float>(jitter);
  imu_bottom_jitter_publisher_->publish(out);
}

void TakeHome::imu_vectornav_callback(vectornav_msgs::msg::CommonGroup::ConstSharedPtr msg) {
  const double jitter = update_jitter(imu_vectornav_timestamps_, rclcpp::Time(msg->header.stamp));
  std_msgs::msg::Float32 out;
  out.data = static_cast<float>(jitter);
  imu_vectornav_jitter_publisher_->publish(out);
}

void TakeHome::curvilinear_distance_callback(std_msgs::msg::Float32::ConstSharedPtr msg) {
  const double current_distance = static_cast<double>(msg->data);
  const rclcpp::Time now = this->now();

  if (!lap_time_initialized_) {
    lap_time_initialized_ = true;
    last_lap_start_time_ = now;
    last_curvilinear_distance_ = current_distance;
    return;
  }

  if (current_distance + 1.0 < last_curvilinear_distance_) {
    const double lap_time_sec = (now - last_lap_start_time_).seconds();
    std_msgs::msg::Float32 out;
    out.data = static_cast<float>(lap_time_sec);
    lap_time_publisher_->publish(out);
    last_lap_start_time_ = now;
  }

  last_curvilinear_distance_ = current_distance;
}

void TakeHome::publish_slip_metrics() {
  if (!odom_received_ || !wheel_speed_received_ || !steering_received_) {
    return;
  }

  constexpr double kWf = 1.638;
  constexpr double kWr = 1.523;
  constexpr double kLf = 1.7238;

  const double vx_rr = vx_ - 0.5 * yaw_rate_ * kWr;
  const double vx_rl = vx_ + 0.5 * yaw_rate_ * kWr;

  const double vx_fr = vx_ - 0.5 * yaw_rate_ * kWf;
  const double vy_fr = vy_ + yaw_rate_ * kLf;
  const double vx_fr_delta = std::cos(steering_angle_rad_) * vx_fr - std::sin(steering_angle_rad_) * vy_fr;

  const double vx_fl = vx_ + 0.5 * yaw_rate_ * kWf;
  const double vy_fl = vy_ + yaw_rate_ * kLf;
  const double vx_fl_delta = std::cos(steering_angle_rad_) * vx_fl - std::sin(steering_angle_rad_) * vy_fl;

  auto compute_slip = [](double wheel_speed, double vx) {
    constexpr double kEpsLocal = 1e-3;
    if (std::fabs(vx) < kEpsLocal) {
      return 0.0;
    }
    return (wheel_speed - vx) / vx;
  };

  std_msgs::msg::Float32 msg_rr;
  std_msgs::msg::Float32 msg_rl;
  std_msgs::msg::Float32 msg_fl;
  std_msgs::msg::Float32 msg_fr;

  msg_rr.data = static_cast<float>(compute_slip(wheel_rr_, vx_rr));
  msg_rl.data = static_cast<float>(compute_slip(wheel_rl_, vx_rl));
  msg_fl.data = static_cast<float>(compute_slip(wheel_fl_, vx_fl_delta));
  msg_fr.data = static_cast<float>(compute_slip(wheel_fr_, vx_fr_delta));

  slip_rr_publisher_->publish(msg_rr);
  slip_rl_publisher_->publish(msg_rl);
  slip_fl_publisher_->publish(msg_fl);
  slip_fr_publisher_->publish(msg_fr);
}


RCLCPP_COMPONENTS_REGISTER_NODE(TakeHome)
