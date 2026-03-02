#include "take_home_node/take_home.hpp"
#include <rclcpp_components/register_node_macro.hpp>
#include <cmath> 


TakeHome::TakeHome(const rclcpp::NodeOptions& options)
    : Node("take_home_metrics", options) {

    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    
    // Subscribers
    odometry_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "vehicle/uva_odometry", qos_profile,
      std::bind(&TakeHome::odometry_callback, this, std::placeholders::_1));

    wheel_speed_subscriber_ = this->create_subscription<raptor_dbw_msgs::msg::WheelSpeedReport>(
      "/raptor_dbw_interface/wheel_speed_report", qos_profile,
      std::bind(&TakeHome::wheel_speed_callback, this, std::placeholders::_1));

    steering_subscriber_ = this->create_subscription<raptor_dbw_msgs::msg::SteeringExtendedReport>(
      "/raptor_dbw_interface/steering_extended_report", qos_profile,
      std::bind(&TakeHome::steering_callback, this, std::placeholders::_1));

    top_imu_subscriber_ = this->create_subscription<novatel_oem7_msgs::msg::RAWIMU>(
      "/novatel_top/rawimu", qos_profile,
      std::bind(&TakeHome::top_imu_callback, this, std::placeholders::_1));

    bottom_imu_subscriber_ = this->create_subscription<novatel_oem7_msgs::msg::RAWIMU>(
      "/novatel_bottom/rawimu", qos_profile,
      std::bind(&TakeHome::bottom_imu_callback, this, std::placeholders::_1));

    vectornav_subscriber_ = this->create_subscription<vectornav_msgs::msg::CommonGroup>(
      "/vectornav/raw/common", qos_profile,
      std::bind(&TakeHome::vectornav_callback, this, std::placeholders::_1));

    curvilinear_subscriber_ = this->create_subscription<std_msgs::msg::Float32>(
      "/curvilinear_distance", qos_profile,
      std::bind(&TakeHome::curvilinear_callback, this, std::placeholders::_1));

    // Publishers
    pub_slip_rr_ = this->create_publisher<std_msgs::msg::Float32>("slip/long/rr", qos_profile);
    pub_slip_rl_ = this->create_publisher<std_msgs::msg::Float32>("slip/long/rl", qos_profile);
    pub_slip_fr_ = this->create_publisher<std_msgs::msg::Float32>("slip/long/fr", qos_profile);
    pub_slip_fl_ = this->create_publisher<std_msgs::msg::Float32>("slip/long/fl", qos_profile);

    pub_jitter_top_ = this->create_publisher<std_msgs::msg::Float32>("imu_top/jitter", qos_profile);
    pub_jitter_bottom_ = this->create_publisher<std_msgs::msg::Float32>("imu_bottom/jitter", qos_profile);
    pub_jitter_vectornav_ = this->create_publisher<std_msgs::msg::Float32>("imu_vectornav/jitter", qos_profile);

    pub_lap_time_ = this->create_publisher<std_msgs::msg::Float32>("lap_time", qos_profile);
}

void TakeHome::wheel_speed_callback(raptor_dbw_msgs::msg::WheelSpeedReport::ConstSharedPtr ws_msg) {
    wheel_speed_fl_ = ws_msg->front_left / 3.6;
    wheel_speed_fr_ = ws_msg->front_right / 3.6;
    wheel_speed_rl_ = ws_msg->rear_left / 3.6;
    wheel_speed_rr_ = ws_msg->rear_right / 3.6;
}

void TakeHome::steering_callback(raptor_dbw_msgs::msg::SteeringExtendedReport::ConstSharedPtr steer_msg) {
    float steering_deg = steer_msg->primary_steering_angle_fbk / 15.0;
    steering_angle_rad_ = steering_deg * (M_PI / 180.0);
}

void TakeHome::odometry_callback(nav_msgs::msg::Odometry::ConstSharedPtr odom_msg) {

    latest_odom_time_ = odom_msg->header.stamp.sec + (odom_msg->header.stamp.nanosec * 1e-9);

    float v_x = odom_msg->twist.twist.linear.x;
    float v_y = odom_msg->twist.twist.linear.y;
    float omega = odom_msg->twist.twist.angular.z;
    float delta = steering_angle_rad_;

    if (std::abs(v_x) < 0.1) return; 

    std_msgs::msg::Float32 msg_rr, msg_rl, msg_fr, msg_fl;

    //  Rear Right 
    float vx_rr = v_x - 0.5 * omega * w_r;
    msg_rr.data = (wheel_speed_rr_ - vx_rr) / vx_rr;
    pub_slip_rr_->publish(msg_rr);

    // Rear Left
    float vx_rl = v_x + 0.5 * omega * w_r;
    msg_rl.data = (wheel_speed_rl_ - vx_rl) / vx_rl;
    pub_slip_rl_->publish(msg_rl);

    // Front Right 
    float vx_fr = v_x - 0.5 * omega * w_f;
    float vy_fr = v_y + omega * l_f;
    float vx_fr_delta = std::cos(delta) * vx_fr - std::sin(delta) * vy_fr;
    
    if (std::abs(vx_fr_delta) > 0.01) {
        msg_fr.data = (wheel_speed_fr_ - vx_fr_delta) / vx_fr_delta;
        pub_slip_fr_->publish(msg_fr);
    }

    //  Front Left 
    float vx_fl = v_x + 0.5 * omega * w_f;
    float vy_fl = v_y + omega * l_f;
    float vx_fl_delta = std::cos(delta) * vx_fl - std::sin(delta) * vy_fl;
    
    if (std::abs(vx_fl_delta) > 0.01) {
        msg_fl.data = (wheel_speed_fl_ - vx_fl_delta) / vx_fl_delta;
        pub_slip_fl_->publish(msg_fl);
    }
}

void TakeHome::top_imu_callback(novatel_oem7_msgs::msg::RAWIMU::ConstSharedPtr msg) {
    double current_time = msg->header.stamp.sec + (msg->header.stamp.nanosec * 1e-9);
    
    float jitter_val = 0.0;
    if (top_imu_window_.calculate_jitter(current_time, jitter_val)) {
        std_msgs::msg::Float32 out_msg;
        out_msg.data = jitter_val;
        pub_jitter_top_->publish(out_msg);
    }
}

void TakeHome::bottom_imu_callback(novatel_oem7_msgs::msg::RAWIMU::ConstSharedPtr msg) {
    double current_time = msg->header.stamp.sec + (msg->header.stamp.nanosec * 1e-9);
    
    float jitter_val = 0.0;
    if (bottom_imu_window_.calculate_jitter(current_time, jitter_val)) {
        std_msgs::msg::Float32 out_msg;
        out_msg.data = jitter_val;
        pub_jitter_bottom_->publish(out_msg);
    }
}

void TakeHome::vectornav_callback(vectornav_msgs::msg::CommonGroup::ConstSharedPtr msg) {
    double current_time = msg->header.stamp.sec + (msg->header.stamp.nanosec * 1e-9);
    
    float jitter_val = 0.0;
    if (vectornav_window_.calculate_jitter(current_time, jitter_val)) {
        std_msgs::msg::Float32 out_msg;
        out_msg.data = jitter_val;
        pub_jitter_vectornav_->publish(out_msg);
    }
}

void TakeHome::curvilinear_callback(std_msgs::msg::Float32::ConstSharedPtr msg) {
    float current_s = msg->data;

    if (latest_odom_time_ < 0.0) return; 

    if (latest_odom_time_ < lap_start_time_) {
        lap_start_time_ = -1.0; 
        last_s_ = -1.0;                 
        return;                              
    }

    if (last_s_ >= 0.0) {
        // change if needed 
        if (current_s < last_s_ - 10.0) { 
            
            if (lap_start_time_ > 0.0) {
                std_msgs::msg::Float32 lap_time_msg;
                lap_time_msg.data = static_cast<float>(latest_odom_time_ - lap_start_time_);
                pub_lap_time_->publish(lap_time_msg);
            }
            
            lap_start_time_ = latest_odom_time_;
        }
    }

    last_s_ = current_s;
}

RCLCPP_COMPONENTS_REGISTER_NODE(TakeHome)
