#include "take_home_node/take_home.hpp"
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

      metric_publisher_ = this->create_publisher<std_msgs::msg::Float32>("metrics_output", qos_profile);
      // Lap time subscriber and publisher
    curvilinear_subscriber_ = this->create_subscription<std_msgs::msg::Float32>(
      "curvilinear_distance", qos_profile,
      std::bind(&TakeHome::curvilinear_distance_callback, this, std::placeholders::_1));
    
    lap_time_publisher_ = this->create_publisher<std_msgs::msg::Float32>("lap_time", qos_profile);
    
    // Initialize lap start time
    lap_start_time_ = this->now();
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
}

void TakeHome::curvilinear_distance_callback(std_msgs::msg::Float32::ConstSharedPtr msg) {
  double current_distance = msg->data;
  
  // Detect lap completion: distance drops significantly
  if (lap_in_progress_ && last_curvilinear_distance_ > 100.0 && current_distance < 50.0) {
    // Lap completed!
    rclcpp::Time lap_end_time = this->now();
    double lap_time_seconds = (lap_end_time - lap_start_time_).seconds();
    
    // Publish lap time
    std_msgs::msg::Float32 lap_time_msg;
    lap_time_msg.data = lap_time_seconds;
    lap_time_publisher_->publish(lap_time_msg);
    
    RCLCPP_INFO(this->get_logger(), "Lap completed! Time: %.2f seconds", lap_time_seconds);
    
    // Reset for next lap
    lap_start_time_ = lap_end_time;
  }
  
  // Start tracking after first message
  if (!lap_in_progress_ && current_distance > 10.0) {
    lap_in_progress_ = true;
    lap_start_time_ = this->now();
  }
  
  last_curvilinear_distance_ = current_distance;
}

RCLCPP_COMPONENTS_REGISTER_NODE(TakeHome)
