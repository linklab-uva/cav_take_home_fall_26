#include "take_home_node/take_home.hpp"
#include <rclcpp_components/register_node_macro.hpp>
#include <vector>

TakeHome::TakeHome(const rclcpp::NodeOptions& options)
    : Node("take_home_metrics", options) {

    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    
    // Look at the hpp file to define all class variables, including subscribers
    // A subscriber will "listen" to a topic and whenever a message is published to it, the subscriber
    // will pass it onto the attached callback (`TakeHome::odometry_callback` in this instance)
    odometry_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "vehicle/uva_odometry", qos_profile,
      std::bind(&TakeHome::odometry_callback, this, std::placeholders::_1));

    wheelSpeed_subscriber_ = this->create_subscription<raptor_dbw_msgs::msg::WheelSpeedReport>(
      "raptor_dbw_interface/wheel_speed_report", qos_profile,
      std::bind(&TakeHome::wheel_speed_callback, this, std::placeholders::_1));

    steering_subscriber_ = this->create_subscription<raptor_dbw_msgs::msg::SteeringExtendedReport>(
      "raptor_dbw_interface/steering_extended_report", qos_profile,
      std::bind(&TakeHome::steering_callback, this, std::placeholders::_1));

    topIMU_subscriber_ = this->create_subscription<novatel_oem7_msgs::msg::RAWIMU>(
      "novatel_top/rawimu", qos_profile,
      std::bind(&TakeHome::top_imu_callback, this, std::placeholders::_1));

    bottomIMU_subscriber_ = this->create_subscription<novatel_oem7_msgs::msg::RAWIMU>(
      "novatel_bottom/rawimu", qos_profile,
      std::bind(&TakeHome::bottom_imu_callback, this, std::placeholders::_1));

    vectorNavIMU_subscriber_ = this->create_subscription<vectornav_msgs::msg::CommonGroup>(
      "vectornav/raw/common", qos_profile,
      std::bind(&TakeHome::vector_nav_callback, this, std::placeholders::_1));

    curvilinear_subscriber_ = this->create_subscription<std_msgs::msg::Float32>(
      "curvilinear_distance", qos_profile,
      std::bind(&TakeHome::curvilinear_callback, this, std::placeholders::_1));

      metric_publisher_ = this->create_publisher<std_msgs::msg::Float32>("metrics_output", qos_profile);
      wheelSlip_rr_publisher_ = this->create_publisher<std_msgs::msg::Float32>("slip/long/rr", qos_profile);
      wheelSlip_rl_publisher_ = this->create_publisher<std_msgs::msg::Float32>("slip/long/rl", qos_profile);
      wheelSlip_fr_publisher_ = this->create_publisher<std_msgs::msg::Float32>("slip/long/fr", qos_profile);
      wheelSlip_fl_publisher_ = this->create_publisher<std_msgs::msg::Float32>("slip/long/fl", qos_profile);
      topIMU_jitter_publisher_ = this->create_publisher<std_msgs::msg::Float32>("imu_top/jitter", qos_profile);
      bottomIMU_jitter_publisher_ = this->create_publisher<std_msgs::msg::Float32>("imu_bottom/jitter", qos_profile);
      navIMU_jitter_publisher_ = this->create_publisher<std_msgs::msg::Float32>("imu_vectornav/jitter", qos_profile);
      laptime_publisher_ = this->create_publisher<std_msgs::msg::Float32>("laptime", qos_profile);
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
  float position_x = odom_msg->pose.pose.position.x;
  float position_y = odom_msg->pose.pose.position.y;
  float position_z = odom_msg->pose.pose.position.z;

  // Do stuff with this callback! or more, idc
  std_msgs::msg::Float32 metric_msg;
  metric_msg.data = (position_x + position_y + position_z) / (position_x + position_z); // Example metric calculation
  metric_publisher_->publish(metric_msg);

  vx = odom_msg->twist.twist.linear.x;
  vy = odom_msg->twist.twist.linear.y;
  w = odom_msg->twist.twist.angular.z;

  vx_rr = vx-0.5*w*1.523;
  k_rr = (latest_wheel_speed_rr - vx_rr) / vx_rr; //wheel slip rear right
  std_msgs::msg::Float32 rr_msg;
  rr_msg.data = k_rr;

  vx_rl = vx+0.5*w*1.523;
  k_rl = (latest_wheel_speed_rl - vx_rl) / vx_rl; //wheel slip rear left
  std_msgs::msg::Float32 rl_msg;
  rl_msg.data = k_rl;

  vx_fr = vx-0.5*w*1.638;
  vy_fr = vy+w*1.7238;
  vx_delta_fr = cos(primary_steering_angle_fbk/15.0f)*vx_fr-sin(primary_steering_angle_fbk/15.0f)*vy_fr;
  k_fr = (latest_wheel_speed_fr - vx_delta_fr) / vx_delta_fr; //wheel slip front right
  std_msgs::msg::Float32 fr_msg;
  fr_msg.data = k_fr;

  vx_fl = vx + 0.5*w*1.638;
  vy_fl = vy+w*1.7238;
  vx_delta_fl = cos(primary_steering_angle_fbk/15.0f)*vx_fl-sin(primary_steering_angle_fbk/15.0f)*vy_fl;
  k_fl = (latest_wheel_speed_fl - vx_delta_fl) / vx_delta_fl; //wheel slip front left
  std_msgs::msg::Float32 fl_msg;
  fl_msg.data = k_fl;

  wheelSlip_rr_publisher_->publish(rr_msg);
  wheelSlip_rl_publisher_->publish(rl_msg);
  wheelSlip_fr_publisher_->publish(fr_msg);
  wheelSlip_fl_publisher_->publish(fl_msg);
}

void TakeHome::wheel_speed_callback(raptor_dbw_msgs::msg::WheelSpeedReport::ConstSharedPtr wheel_speed_msg) {
  latest_wheel_speed_rr = wheel_speed_msg->rear_right*(1000.0f/3600.0f);
  latest_wheel_speed_rl = wheel_speed_msg->rear_left*(1000.0f/3600.0f);
  latest_wheel_speed_fr = wheel_speed_msg->front_right*(1000.0f/3600.0f);
  latest_wheel_speed_fl = wheel_speed_msg->front_left*(1000.0f/3600.0f);
}

void TakeHome::steering_callback(raptor_dbw_msgs::msg::SteeringExtendedReport::ConstSharedPtr steering_msg) {
  primary_steering_angle_fbk = steering_msg->primary_steering_angle_fbk;
}

double TakeHome::calc_jitter(std::deque<uint64_t> &timestamps, uint64_t newTime) {
  timestamps.push_back(newTime);

  while(!timestamps.empty() && (newTime - timestamps.front()) > 1e9){
    timestamps.pop_front();
  }

  if (timestamps.size() < 2){
    return 0.0;
  }

  std::vector<double> deltas;
  for (size_t i = 1; i < timestamps.size(); ++i) {
    double delta = (timestamps[i] - timestamps[i-1]) / 1e6;
    deltas.push_back(delta);
  }

  double mean_delta = 0.0;
  for (double d : deltas) {
    mean_delta += d;
  }
  mean_delta /= deltas.size();

  double sum_squared_diffs = 0.0;
  for (double d : deltas) {
    double diff = d - mean_delta;
    sum_squared_diffs += diff * diff;
  }
  double jitter = std::sqrt(sum_squared_diffs / deltas.size());
  return jitter;
}

void TakeHome::top_imu_callback(novatel_oem7_msgs::msg::RAWIMU::ConstSharedPtr top_imu_msg) {
  uint64_t timeStamp = rclcpp::Time(top_imu_msg->header.stamp).nanoseconds();

  std_msgs::msg::Float32 jitter_msg;
  jitter_msg.data = calc_jitter(top_timestamps_, timeStamp);
  topIMU_jitter_publisher_->publish(jitter_msg);
}

void TakeHome::bottom_imu_callback(novatel_oem7_msgs::msg::RAWIMU::ConstSharedPtr bottom_imu_msg) {
  uint64_t timeStamp = rclcpp::Time(bottom_imu_msg->header.stamp).nanoseconds();

  std_msgs::msg::Float32 jitter_msg;
  jitter_msg.data = calc_jitter(bottom_timestamps_, timeStamp);
  bottomIMU_jitter_publisher_->publish(jitter_msg);
}

void TakeHome::vector_nav_callback(vectornav_msgs::msg::CommonGroup::ConstSharedPtr vectornav_msg) {
  uint64_t timeStamp = rclcpp::Time(vectornav_msg->header.stamp).nanoseconds();

  std_msgs::msg::Float32 jitter_msg;
  jitter_msg.data = calc_jitter(nav_timestamps_, timeStamp);
  navIMU_jitter_publisher_->publish(jitter_msg);
}

void TakeHome::curvilinear_callback(std_msgs::msg::Float32::ConstSharedPtr curvilinear_msg) {
  double current_distance = curvilinear_msg->data;
  uint64_t current_time = this->now().nanoseconds();

  if (current_distance < last_curvilinear_distance){
    if (last_lap_time != 0){
      double lap_time = (current_time - last_lap_time) / 1e9; 
      std_msgs::msg::Float32 lap_time_msg;
      lap_time_msg.data = lap_time;
      laptime_publisher_->publish(lap_time_msg);
    }
    last_lap_time = current_time;
  }
  last_curvilinear_distance = current_distance;
}


RCLCPP_COMPONENTS_REGISTER_NODE(TakeHome)
