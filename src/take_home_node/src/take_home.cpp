#include "take_home_node/take_home.hpp"
#include <rclcpp_components/register_node_macro.hpp>
#include <cmath>
#include <vector>

TakeHome::TakeHome(const rclcpp::NodeOptions& options)
    : Node("take_home_metrics", options) {

    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    
  auto reliable_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    // Look at the hpp file to define all class variables, including subscribers
    // A subscriber will "listen" to a topic and whenever a message is published to it, the subscriber
    // will pass it onto the attached callback (`TakeHome::odometry_callback` in this instance)

    //subscriptions(type, topic, qos_profile, std::bind(function,this,std::placeholders_1))
    /** A: subscribers and publishers */
    odometry_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>( 
      "vehicle/uva_odometry", qos_profile,
      std::bind(&TakeHome::odometry_callback, this, std::placeholders::_1));
    wheel_speed_subscriber_ = this->create_subscription<raptor_dbw_msgs::msg::WheelSpeedReport>(
      "raptor_dbw_interface/wheel_speed_report", qos_profile,
      std::bind(&TakeHome::wheel_speed_callback, this, std::placeholders::_1));
    steering_subscriber_ = this->create_subscription<raptor_dbw_msgs::msg::SteeringExtendedReport>(
      "raptor_dbw_interface/steering_extended_report", qos_profile,
      std::bind(&TakeHome::steering_callback, this, std::placeholders::_1));

    rr_slip_publisher_ = this->create_publisher<std_msgs::msg::Float32>("slip/long/rr", qos_profile);
    rl_slip_publisher_ = this->create_publisher<std_msgs::msg::Float32>("slip/long/rl", qos_profile);
    fr_slip_publisher_ = this->create_publisher<std_msgs::msg::Float32>("slip/long/fr", qos_profile);
    fl_slip_publisher_ = this->create_publisher<std_msgs::msg::Float32>("slip/long/fl", qos_profile);

     /** B: subscribers and publishers */
    top_imu_subscriber_ = this->create_subscription<novatel_oem7_msgs::msg::RAWIMU>(
      "novatel_top/rawimu", reliable_qos,
      std::bind(&TakeHome::top_imu_callback, this, std::placeholders::_1));
    bottom_imu_subscriber_ = this->create_subscription<novatel_oem7_msgs::msg::RAWIMU>(
    "novatel_bottom/rawimu", reliable_qos,
      std::bind(&TakeHome::bottom_imu_callback, this, std::placeholders::_1));
      vectornav_imu_subscriber_ = this->create_subscription<vectornav_msgs::msg::CommonGroup>(
       "vectornav/raw/common", reliable_qos,
       std::bind(&TakeHome::vectornav_imu_callback, this, std::placeholders::_1));

    top_imu_publisher_ = this->create_publisher<std_msgs::msg::Float32>("imu_top/jitter", qos_profile);
    bottom_imu_publisher_ = this->create_publisher<std_msgs::msg::Float32>("imu_bottom/jitter", qos_profile);
    vectornav_imu_publisher_ = this->create_publisher<std_msgs::msg::Float32>("imu_vectornav/jitter", qos_profile);

    /** C: subscribers and publishers */
    curvilinear_subscriber_ = this->create_subscription<std_msgs::msg::Float32>(
      "curvilinear_distance", qos_profile,
      std::bind(&TakeHome::curvilinear_distance_callback, this, std::placeholders::_1));
    curvilinear_publisher_ = this->create_publisher<std_msgs::msg::Float32>("lap_time", qos_profile);
  
}

// 
/**
 * Whenever a message is published to the topic "vehicle/uva_odometry" the subscriber passes the message onto this callback
 * To see what is in each message look for the corresponding .msg file in this repository
 * For instance, when running `ros2 bag info` on the given bag, we see the wheel speed report has message type of raptor_dbw_msgs/msgs/WheelSpeedReport
 * and from the corresponding WheelSpeedReport.msg file we see that we can do msg->front_left for the front left speed for instance.
 * For the built in ROS2 messages, we can find the documentation online: e.g. https://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html
 */
 // Update vehicle speeds
void TakeHome::odometry_callback(nav_msgs::msg::Odometry::ConstSharedPtr odom_msg) {
  l_longitudinal_speed_ = odom_msg->twist.twist.linear.x;
  yaw_rate_ = odom_msg->twist.twist.angular.z;
  l_lateral_speed_ = odom_msg->twist.twist.linear.y;
  last_odometer_time_ = odom_msg->header.stamp.sec * 1e9 + odom_msg->header.stamp.nanosec;
  get_wheel_slip();
}

/**
A: wheel slip for four wheels

Rear wheels: 
Speed of car at rr = speed of car long linear (m/s) - 0.5 * yaw rate (rad/s) * the distance between left and right tire = 1.523 (m))
Slip ratio rr = (speed of rr wheel (km/hr) - speed of car at rr (m/s)) / speed of car at rr (m/s)

Front wheels:
Speed of car at fr,x = speed of car long linear (m/s) - 0.5 * yaw rate (rad/s) * front track width = 1.638 (m))
Speed of car at fr,y = speed of car lateral linear (m/s) + yaw rate (rad/s)*1.7238)
speed of car with wheel angle = cos(wheel angle)*Speed of car at fr,x - sin(wheel angle)*Speed of car at fr,y (m/s)
slip ratio fr = (speed of fr wheel (km/hr) - speed of car with wheel angle (m/s)) / speed of car with wheel angle (m/s)
*/

// Update the wheel speeds
void TakeHome::wheel_speed_callback(raptor_dbw_msgs::msg::WheelSpeedReport::ConstSharedPtr w_speed_msg){
  fl_speed_ = w_speed_msg->front_left;
  fr_speed_ = w_speed_msg->front_right;
  rl_speed_ = w_speed_msg->rear_left;
  rr_speed_ = w_speed_msg->rear_right;
  last_wheel_speed_time_ = w_speed_msg->header.stamp.sec * 1e9 + w_speed_msg->header.stamp.nanosec;
  get_wheel_slip();
}

//Update wheel angle
void TakeHome::steering_callback(raptor_dbw_msgs::msg::SteeringExtendedReport::ConstSharedPtr steering_msg){
  wheel_angle_ = (steering_msg->primary_steering_angle_fbk) / 15.0;
  last_steering_time_ = steering_msg->header.stamp.sec * 1e9 + steering_msg->header.stamp.nanosec;
  get_wheel_slip();
}

void TakeHome::get_wheel_slip(){
  if (last_steering_time_ == 0 || last_wheel_speed_time_ == 0 || last_odometer_time_ == 0){ //do not have data for some topic
    return; 
  }

  if (abs(last_odometer_time_ - last_wheel_speed_time_) < 50000000 && abs(last_odometer_time_ - last_steering_time_) < 5000000){ //data is within 50ms
    double long_speed_car_rr = l_longitudinal_speed_ - (0.5 * yaw_rate_ * R_TRACK_WIDTH); //long speed of car at rr
    double slip_rr = 0;
    if (long_speed_car_rr > 5.0){
      slip_rr = ((rr_speed_ / 3.6) - long_speed_car_rr) / long_speed_car_rr;
    }
    double long_speed_car_rl = l_longitudinal_speed_ + (0.5 * yaw_rate_ * R_TRACK_WIDTH);
    double slip_rl = 0;
    if (long_speed_car_rl > 5.0){
      slip_rl = ((rl_speed_ / 3.6) - long_speed_car_rl) / long_speed_car_rl;
    }

    double long_speed_car_fr = l_longitudinal_speed_ - (0.5 * yaw_rate_ * F_TRACK_WIDTH);
    double lat_speed_car = l_lateral_speed_ + yaw_rate_ * COG_TO_FRONT_W;
    double speed_car_w_angle_fr = cos(wheel_angle_) * long_speed_car_fr - sin(wheel_angle_) * lat_speed_car;
    double slip_fr = 0;
    if (speed_car_w_angle_fr > 5.0){
      slip_fr = ((fr_speed_ / 3.6) - speed_car_w_angle_fr) / speed_car_w_angle_fr;
    }
    double long_speed_car_fl = l_longitudinal_speed_ + (0.5 * yaw_rate_ * F_TRACK_WIDTH);
    double speed_car_w_angle_fl = cos(wheel_angle_) * long_speed_car_fl - sin(wheel_angle_) * lat_speed_car;
    double slip_fl = 0;
    if (speed_car_w_angle_fl > 5.0){
      slip_fl = ((fl_speed_ / 3.6) - speed_car_w_angle_fl) / speed_car_w_angle_fl;
    }

    std_msgs::msg::Float32 slip_rr_msg;
    slip_rr_msg.data = slip_rr;
    rr_slip_publisher_->publish(slip_rr_msg);

    std_msgs::msg::Float32 slip_rl_msg;
    slip_rl_msg.data = slip_rl;
    rl_slip_publisher_->publish(slip_rl_msg);

    std_msgs::msg::Float32 slip_fr_msg;
    slip_fr_msg.data = slip_fr;
    fr_slip_publisher_->publish(slip_fr_msg);

    std_msgs::msg::Float32 slip_fl_msg;
    slip_fl_msg.data = slip_fl;
    fl_slip_publisher_->publish(slip_fl_msg);
  }
}

/**
B: jitter data
store time data in an iterable queue. If adding to the queue creates a difference greater than 1 s from
start to finish, then dequeue until this is not so.
Jitter is measured in nanoseconds
 */

void TakeHome::top_imu_callback(novatel_oem7_msgs::msg::RAWIMU::ConstSharedPtr top_imu_msg){
  int64_t time_stamp = top_imu_msg->header.stamp.sec * 1e9 + top_imu_msg->header.stamp.nanosec;
  top_imu_.push_back(time_stamp);
  if (top_imu_.size() < 2) return;

  if (time_stamp - top_imu_.front() > 1000000000){ // move the window, but first calculate jitter
    double jitter = calculate_jitter(top_imu_);
    while(time_stamp - top_imu_.front() > 1000000000){
      top_imu_.pop_front();
    }
    std_msgs::msg::Float32 jitter_msg;
    jitter_msg.data = jitter;
    top_imu_publisher_->publish(jitter_msg);
  }
}

void TakeHome::bottom_imu_callback(novatel_oem7_msgs::msg::RAWIMU::ConstSharedPtr bottom_imu_msg){
  int64_t time_stamp = bottom_imu_msg->header.stamp.sec * 1e9 + bottom_imu_msg->header.stamp.nanosec;
  bottom_imu_.push_back(time_stamp);
  if (bottom_imu_.size() < 2) return;

  if (time_stamp - bottom_imu_.front() > 1000000000){ // move the window, but first calculate jitter
    double jitter = calculate_jitter(bottom_imu_);
    while(time_stamp - bottom_imu_.front() > 1000000000){
      bottom_imu_.pop_front();
    }
    std_msgs::msg::Float32 jitter_msg;
    jitter_msg.data = jitter;
    bottom_imu_publisher_->publish(jitter_msg);
  }
}

void TakeHome::vectornav_imu_callback(vectornav_msgs::msg::CommonGroup::ConstSharedPtr vectornav_imu_msg){
  int64_t time_stamp = vectornav_imu_msg->header.stamp.sec * 1e9 +vectornav_imu_msg->header.stamp.nanosec;
  vectornav_imu_.push_back(time_stamp);
  if (vectornav_imu_.size() < 2) return;

  if (time_stamp - vectornav_imu_.front() > 1000000000){ // move the window, but first calculate jitter
    double jitter = calculate_jitter(vectornav_imu_);
    while(time_stamp - vectornav_imu_.front() > 1000000000){
      vectornav_imu_.pop_front();
    }
    std_msgs::msg::Float32 jitter_msg;
    jitter_msg.data = jitter;
    vectornav_imu_publisher_->publish(jitter_msg);
  }
}

//calculate the jitter for the 1s in the queue
double TakeHome::calculate_jitter(std::deque<int64_t>& imu){
  std::vector<double> delta_times;
  double sum_deltas = 0.0;
  for (size_t i = 1; i < imu.size(); i++){
    delta_times.push_back(imu[i] - imu[i-1]);
    sum_deltas += imu[i] - imu[i-1];
  } 
  double avg_delta = sum_deltas / delta_times.size();

  double variance = 0.0;
  for (size_t j = 0; j < delta_times.size(); j++){
    variance += (delta_times[j] - avg_delta) * (delta_times[j] - avg_delta);
  }
  return variance / delta_times.size();
}

/** C: calculate the lap time. When the curvilinear distance drops off by ~3000, the lap has been completed, at this moment take the time of the first 
data from the last data, and publish this time
 */

void TakeHome::curvilinear_distance_callback(std_msgs::msg::Float32::ConstSharedPtr curvilinear_msg){
  if (start_time_ == 0 && curvilinear_msg->data < 1 && last_odometer_time_ != 0){
    start_time_ = last_odometer_time_;
  }
  if (lap_distance_ - curvilinear_msg->data > 2000){  
    double lap_time = (last_odometer_time_ - start_time_) / 1e9;
    std_msgs::msg::Float32 lap_time_msg;
    lap_time_msg.data = lap_time;
    curvilinear_publisher_->publish(lap_time_msg);
    start_time_ = 0;
  }
  lap_distance_ = curvilinear_msg->data;
 
}


RCLCPP_COMPONENTS_REGISTER_NODE(TakeHome)
