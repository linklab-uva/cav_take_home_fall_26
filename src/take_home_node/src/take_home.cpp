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
      std::bind(&TakeHome::odometry_callback, this, std::placeholders::_1)
    );

    wheel_speed_subscriber_ = this->create_subscription<raptor_dbw_msgs::msg::WheelSpeedReport>(
        "/raptor_dbw_interface/wheel_speed_report", qos_profile,
        std::bind(&TakeHome::wheel_speed_callback, this, std::placeholders::_1)
    );
    steering_subscriber_ = this->create_subscription<raptor_dbw_msgs::msg::SteeringExtendedReport>(
        "raptor_dbw_interface/steering_extended_report", qos_profile,
        std::bind(&TakeHome::steering_callback, this, std::placeholders::_1)
    );

    metric_publisher_ = this->create_publisher<std_msgs::msg::Float32>("metrics_output", qos_profile);
    slip_long_rr_publisher_ = this->create_publisher<std_msgs::msg::Float32>("slip/long/rr", qos_profile);
    slip_long_rl_publisher_ = this->create_publisher<std_msgs::msg::Float32>("slip/long/rl", qos_profile);
    slip_long_fr_publisher_ = this->create_publisher<std_msgs::msg::Float32>("slip/long/fr", qos_profile);
    slip_long_fl_publisher_ = this->create_publisher<std_msgs::msg::Float32>("slip/long/fl", qos_profile);

    // IMU ---------------------------------------------------------------------------------------------------

    top_imu_subscriber_ = this->create_subscription<novatel_oem7_msgs::msg::RAWIMU>(
        "/novatel_top/rawimu", qos_profile,
        std::bind(&TakeHome::top_imu_callback, this, std::placeholders::_1)
    );
    bottom_imu_subscriber_ = this->create_subscription<novatel_oem7_msgs::msg::RAWIMU>(
        "/novatel_bottom/rawimu", qos_profile,
        std::bind(&TakeHome::bottom_imu_callback, this, std::placeholders::_1)
    );
    vectornav_imu_subscriber_ = this->create_subscription<vectornav_msgs::msg::CommonGroup>(
        "/vectornav/raw/common", qos_profile,
        std::bind(&TakeHome::vectornav_imu_callback, this, std::placeholders::_1)
    );

    top_imu_jitter_publisher_ = this->create_publisher<std_msgs::msg::Float32>("imu_top/jitter", qos_profile);
    bottom_imu_jitter_publisher_ = this->create_publisher<std_msgs::msg::Float32>("imu_bottom/jitter", qos_profile);
    vectornav_imu_jitter_publisher_ = this->create_publisher<std_msgs::msg::Float32>("imu_vectornav/jitter", qos_profile);

    // lap time ----------------------------------------------------------------------------------------------

    curvilinear_distance_subscriber_ = this->create_subscription<std_msgs::msg::Float32>(
        "/curvilinear_distance", qos_profile,
        std::bind(&TakeHome::curvilinear_distance_callback, this, std::placeholders::_1)
    );

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
    received_odometry_ = true;
  float position_x = odom_msg->pose.pose.position.x;
  float position_y = odom_msg->pose.pose.position.y;
  float position_z = odom_msg->pose.pose.position.z;

  // Do stuff with this callback! or more, idc
  std_msgs::msg::Float32 metric_msg;
  metric_msg.data = (position_x + position_y + position_z) / (position_x + position_z); // Example metric calculation
  metric_publisher_->publish(metric_msg);

    vx_ = odom_msg->twist.twist.linear.x; // TODO: x component or whole vector?
    vy_ = odom_msg->twist.twist.linear.y;
    angular_velocity_ = odom_msg->twist.twist.angular.z; // yaw rate (rotation wrt vertical axis)

    publish_wheel_slip();
}

void TakeHome::wheel_speed_callback(raptor_dbw_msgs::msg::WheelSpeedReport::ConstSharedPtr wheel_speed_msg) {
    received_wheel_speed_ = true;

    static const double KPH_TO_MPS = 1000.0 / 3600.0;

    wheel_speed_fl_ = wheel_speed_msg->front_left * KPH_TO_MPS;
    wheel_speed_fr_ = wheel_speed_msg->front_right * KPH_TO_MPS;
    wheel_speed_rl_ = wheel_speed_msg->rear_left * KPH_TO_MPS;
    wheel_speed_rr_ = wheel_speed_msg->rear_right * KPH_TO_MPS;
}

void TakeHome::steering_callback(raptor_dbw_msgs::msg::SteeringExtendedReport::ConstSharedPtr steering_msg) {
    received_steering_data_ = true;
    primary_steering_angle_fbk_ = steering_msg->primary_steering_angle_fbk / 15.0;
}

void TakeHome::publish_wheel_slip() {
    if (!(received_odometry_ && received_steering_data_ && received_wheel_speed_)) return;

    if (std::abs(vx_) < 1e-3) { // don't publish if car is stopped to prevent division by 0 error
        return;
    }

    // rear right
    std_msgs::msg::Float32 rr_msg;
    double vx_rr = vx_ - (0.5*angular_velocity_*rear_track_width_);
    rr_msg.data = (wheel_speed_rr_ - vx_rr)/vx_rr;

    // rear left
    std_msgs::msg::Float32 rl_msg;
    double vx_rl = vx_ + (0.5*angular_velocity_*rear_track_width_);
    rl_msg.data = (wheel_speed_rl_ - vx_rl)/vx_rl;

    // front right
    std_msgs::msg::Float32 fr_msg;
    double vx_fr = vx_ - (0.5*angular_velocity_*front_track_width_);
    double vy_fr = vy_ + (angular_velocity_*dist_COG_front_);
    double delta = primary_steering_angle_fbk_ * 2 * M_PI / 360;
    double vx_fr_delta = (cos(delta) * vx_fr) - (sin(delta)*vy_fr);
    fr_msg.data = (wheel_speed_fr_ - vx_fr_delta)/vx_fr_delta;

    // front left
    std_msgs::msg::Float32 fl_msg;
    double vx_fl = vx_ + (0.5*angular_velocity_*front_track_width_);
    double vy_fl = vy_ + (angular_velocity_*dist_COG_front_);
    double vx_fl_delta = (cos(delta) * vx_fl) - (sin(delta)*vy_fl);
    fl_msg.data = (wheel_speed_fl_ - vx_fl_delta)/vx_fl_delta;

    // publish
    slip_long_rr_publisher_->publish(rr_msg);
    slip_long_rl_publisher_->publish(rl_msg);
    slip_long_fr_publisher_->publish(fr_msg);
    slip_long_fl_publisher_->publish(fl_msg);
}

double TakeHome::calculate_jitter(std::deque<uint64_t> &timestamps, uint64_t new_timestamp) {
    timestamps.push_back(new_timestamp);

    // maintain 1 second window
    while (!timestamps.empty() && new_timestamp - timestamps.front() > 1e9) {
        timestamps.pop_front();
    }

    if (timestamps.size() < 3) {
        return 0.0; // not enough timestamps yet (need at least 2 intervals)
    }

    double total_dt_sum = timestamps.back() - timestamps.front();
    double num_intervals = timestamps.size() - 1;
    double mean_dt = total_dt_sum / num_intervals;

    // variance computation
    double sum_squares = 0.0;
    for (int i=0;i<num_intervals;i++) {
        double diff = (timestamps[i+1]-timestamps[i]) - mean_dt;
        sum_squares += diff * diff;
    }

    return (sum_squares / num_intervals) / 1e18; // convert from ns^2 to s^2
}

void TakeHome::top_imu_callback(novatel_oem7_msgs::msg::RAWIMU::ConstSharedPtr top_imu_msg) {
    uint64_t timestamp = rclcpp::Time(top_imu_msg->header.stamp).nanoseconds();

    std_msgs::msg::Float32 jitter;
    jitter.data = calculate_jitter(top_imu_timestamps_, timestamp);
    top_imu_jitter_publisher_->publish(jitter);
}

void TakeHome::bottom_imu_callback(novatel_oem7_msgs::msg::RAWIMU::ConstSharedPtr bottom_imu_msg) {
    uint64_t timestamp = rclcpp::Time(bottom_imu_msg->header.stamp).nanoseconds();

    std_msgs::msg::Float32 jitter;
    jitter.data = calculate_jitter(bottom_imu_timestamps_, timestamp);
    bottom_imu_jitter_publisher_->publish(jitter);
}

void TakeHome::vectornav_imu_callback(vectornav_msgs::msg::CommonGroup::ConstSharedPtr vectornav_imu_msg) {
    uint64_t timestamp = rclcpp::Time(vectornav_imu_msg->header.stamp).nanoseconds();

    std_msgs::msg::Float32 jitter;
    jitter.data = calculate_jitter(vectornav_imu_timestamps_, timestamp);
    vectornav_imu_jitter_publisher_->publish(jitter);
}

void TakeHome::curvilinear_distance_callback(std_msgs::msg::Float32::ConstSharedPtr curvilinear_distance_msg) {
    double current_distance = curvilinear_distance_msg->data;
    uint64_t current_time = this->get_clock()->now().nanoseconds();

    if (lap_time_start_ == 0) {
        lap_time_start_ = current_time;
        last_recorded_curvilinear_distance_ = current_distance;
        return;
    }

    if (current_distance < last_recorded_curvilinear_distance_ - 10.0) { // start of new lap
        double lap_duration_s = (double) (current_time - lap_time_start_) / 1e9;

        std_msgs::msg::Float32 current_lap_time;
        current_lap_time.data = lap_duration_s;
        lap_time_publisher_->publish(current_lap_time);

        lap_time_start_ = current_time;
    }

    last_recorded_curvilinear_distance_ = current_distance;
}


RCLCPP_COMPONENTS_REGISTER_NODE(TakeHome)
