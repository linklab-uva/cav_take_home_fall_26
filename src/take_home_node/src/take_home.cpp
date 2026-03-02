#include "take_home_node/take_home.hpp"
#include <rclcpp_components/register_node_macro.hpp>
#include <algorithm>
#include <numeric>

TakeHome::TakeHome(const rclcpp::NodeOptions& options)
    : Node("take_home_metrics", options),
      lap_time_(0.0),
      is_new_lap_(false) {

    // Use SensorDataQoS (BestEffort) for all bag-fed subscriptions to match bag QoS
    auto sensor_qos = rclcpp::SensorDataQoS();
    
    // Existing subscriber and publisher (keep intact)
    odometry_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/vehicle/uva_odometry", sensor_qos,
      std::bind(&TakeHome::odometry_callback, this, std::placeholders::_1));

    metric_publisher_ = this->create_publisher<std_msgs::msg::Float32>("/metrics_output", 10);

    // New subscribers for Task 2 - use sensor QoS to match bag
    wheel_speed_subscriber_ = this->create_subscription<raptor_dbw_msgs::msg::WheelSpeedReport>(
      "/raptor_dbw_interface/wheel_speed_report", sensor_qos,
      std::bind(&TakeHome::wheel_speed_callback, this, std::placeholders::_1));

    steering_subscriber_ = this->create_subscription<raptor_dbw_msgs::msg::SteeringExtendedReport>(
      "/raptor_dbw_interface/steering_extended_report", sensor_qos,
      std::bind(&TakeHome::steering_callback, this, std::placeholders::_1));

    curvilinear_distance_subscriber_ = this->create_subscription<std_msgs::msg::Float32>(
      "/curvilinear_distance", sensor_qos,
      std::bind(&TakeHome::curvilinear_distance_callback, this, std::placeholders::_1));

    imu_top_subscriber_ = this->create_subscription<novatel_oem7_msgs::msg::RAWIMU>(
      "/novatel_top/rawimu", sensor_qos,
      std::bind(&TakeHome::imu_top_callback, this, std::placeholders::_1));

    imu_bottom_subscriber_ = this->create_subscription<novatel_oem7_msgs::msg::RAWIMU>(
      "/novatel_bottom/rawimu", sensor_qos,
      std::bind(&TakeHome::imu_bottom_callback, this, std::placeholders::_1));

    imu_vectornav_subscriber_ = this->create_subscription<vectornav_msgs::msg::CommonGroup>(
      "/vectornav/raw/common", sensor_qos,
      std::bind(&TakeHome::imu_vectornav_callback, this, std::placeholders::_1));

    // New publishers for Task 2 metrics - created in constructor to ensure they're advertised
    // Use simple QoS (depth 10) - NOT conditional, NOT in callbacks, always created
    slip_rr_publisher_ = this->create_publisher<std_msgs::msg::Float32>("/slip/long/rr", 10);
    slip_rl_publisher_ = this->create_publisher<std_msgs::msg::Float32>("/slip/long/rl", 10);
    slip_fr_publisher_ = this->create_publisher<std_msgs::msg::Float32>("/slip/long/fr", 10);
    slip_fl_publisher_ = this->create_publisher<std_msgs::msg::Float32>("/slip/long/fl", 10);
    
    imu_top_jitter_publisher_ = this->create_publisher<std_msgs::msg::Float32>("/imu_top/jitter", 10);
    imu_bottom_jitter_publisher_ = this->create_publisher<std_msgs::msg::Float32>("/imu_bottom/jitter", 10);
    imu_vectornav_jitter_publisher_ = this->create_publisher<std_msgs::msg::Float32>("/imu_vectornav/jitter", 10);

    lap_time_publisher_ = this->create_publisher<std_msgs::msg::Float32>("/lap_time", 10);

    // Timer to publish metrics at regular intervals (20 Hz)
    metrics_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&TakeHome::publish_metrics_timer_callback, this));

    // Publish initial values immediately to advertise topics (ROS2 topics only appear after first publish)
    // This ensures topics are visible in ros2 topic list even before data arrives
    std_msgs::msg::Float32 init_msg;
    init_msg.data = 0.0;
    metric_publisher_->publish(init_msg);  // Advertise metrics_output
    slip_rr_publisher_->publish(init_msg);
    slip_rl_publisher_->publish(init_msg);
    slip_fr_publisher_->publish(init_msg);
    slip_fl_publisher_->publish(init_msg);
    
    RCLCPP_INFO(this->get_logger(), "Take Home Node initialized - publishers created and initial messages published");
}

void TakeHome::odometry_callback(nav_msgs::msg::Odometry::ConstSharedPtr odom_msg) {
  static bool first_odom = true;
  if (first_odom) {
    RCLCPP_INFO(this->get_logger(), "First odometry message received");
    first_odom = false;
  }
  
  latest_odom_ = odom_msg;
  
  float position_x = odom_msg->pose.pose.position.x;
  float position_y = odom_msg->pose.pose.position.y;
  float position_z = odom_msg->pose.pose.position.z;

  // Example metric calculation (keep as-is) - ALWAYS publish this
  std_msgs::msg::Float32 metric_msg;
  float denominator = position_x + position_z;
  if (std::abs(denominator) < EPSILON) {
    metric_msg.data = 0.0;
  } else {
    metric_msg.data = (position_x + position_y + position_z) / denominator;
  }
  metric_publisher_->publish(metric_msg);

  // ALWAYS publish wheel slip ratios when odometry arrives (use fallbacks if wheel/steering data missing)
  auto slip_ratios = compute_wheel_slip_ratios();
  if (slip_ratios.has_value()) {
    auto [k_rr, k_rl, k_fr, k_fl] = slip_ratios.value();

    std_msgs::msg::Float32 msg_rr;
    msg_rr.data = static_cast<float>(k_rr);
    slip_rr_publisher_->publish(msg_rr);

    std_msgs::msg::Float32 msg_rl;
    msg_rl.data = static_cast<float>(k_rl);
    slip_rl_publisher_->publish(msg_rl);

    std_msgs::msg::Float32 msg_fr;
    msg_fr.data = static_cast<float>(k_fr);
    slip_fr_publisher_->publish(msg_fr);

    std_msgs::msg::Float32 msg_fl;
    msg_fl.data = static_cast<float>(k_fl);
    slip_fl_publisher_->publish(msg_fl);
  } else {
    // If odometry is missing, publish 0.0 to keep topics advertised
    std_msgs::msg::Float32 zero_msg;
    zero_msg.data = 0.0;
    slip_rr_publisher_->publish(zero_msg);
    slip_rl_publisher_->publish(zero_msg);
    slip_fr_publisher_->publish(zero_msg);
    slip_fl_publisher_->publish(zero_msg);
  }
}

void TakeHome::wheel_speed_callback(raptor_dbw_msgs::msg::WheelSpeedReport::ConstSharedPtr msg) {
  static bool first_wheel = true;
  if (first_wheel) {
    RCLCPP_INFO(this->get_logger(), "First wheel speed report received");
    first_wheel = false;
  }
  latest_wheel_speed_ = msg;
}

void TakeHome::steering_callback(raptor_dbw_msgs::msg::SteeringExtendedReport::ConstSharedPtr msg) {
  static bool first_steering = true;
  if (first_steering) {
    RCLCPP_INFO(this->get_logger(), "First steering report received");
    first_steering = false;
  }
  latest_steering_ = msg;
}

void TakeHome::curvilinear_distance_callback(std_msgs::msg::Float32::ConstSharedPtr msg) {
  double current_dist = msg->data;
  double current_time = this->now().seconds();

  // Detect lap wrap: if prev_dist > 30.0 and (prev_dist - curr_dist) > 10.0
  if (prev_curvilinear_distance_.has_value()) {
    if (prev_curvilinear_distance_.value() > 30.0 && 
        (prev_curvilinear_distance_.value() - current_dist) > 10.0) {
      // Lap completed
      if (lap_start_time_.has_value()) {
        lap_time_ = current_time - lap_start_time_.value();
        is_new_lap_ = true;
      }
      // Start new lap
      lap_start_time_ = current_time;
    }
  }

  latest_curvilinear_distance_ = current_dist;
  prev_curvilinear_distance_ = current_dist;
}

void TakeHome::imu_top_callback(novatel_oem7_msgs::msg::RAWIMU::ConstSharedPtr msg) {
  add_imu_timestamp(imu_top_timestamps_, msg->header);
}

void TakeHome::imu_bottom_callback(novatel_oem7_msgs::msg::RAWIMU::ConstSharedPtr msg) {
  add_imu_timestamp(imu_bottom_timestamps_, msg->header);
}

void TakeHome::imu_vectornav_callback(vectornav_msgs::msg::CommonGroup::ConstSharedPtr msg) {
  add_imu_timestamp(imu_vectornav_timestamps_, msg->header);
}

void TakeHome::add_imu_timestamp(std::deque<std::pair<double, double>>& timestamp_deque,
                                  const std_msgs::msg::Header& header) {
  double msg_time;
  if (header.stamp.sec > 0 || header.stamp.nanosec > 0) {
    // Use message timestamp
    msg_time = header.stamp.sec + header.stamp.nanosec / 1e9;
  } else {
    // Fallback to node clock
    msg_time = this->now().seconds();
  }

  double current_time = this->now().seconds();

  // Add new timestamp
  timestamp_deque.push_back({current_time, msg_time});

  // Remove timestamps older than 1.0 second
  while (!timestamp_deque.empty() && (current_time - timestamp_deque.front().first) > 1.0) {
    timestamp_deque.pop_front();
  }
}

double TakeHome::compute_imu_jitter(const std::deque<std::pair<double, double>>& timestamp_deque) {
  if (timestamp_deque.size() < 3) {
    return 0.0;
  }

  // Extract message timestamps and sort
  std::vector<double> timestamps;
  for (const auto& pair : timestamp_deque) {
    timestamps.push_back(pair.second);
  }
  std::sort(timestamps.begin(), timestamps.end());

  if (timestamps.size() < 3) {
    return 0.0;
  }

  // Compute consecutive Δt values
  std::vector<double> deltas;
  for (size_t i = 1; i < timestamps.size(); ++i) {
    double dt = timestamps[i] - timestamps[i-1];
    if (dt > 0) {  // Only positive deltas
      deltas.push_back(dt);
    }
  }

  if (deltas.size() < 2) {
    return 0.0;
  }

  // Compute population variance of Δt
  double mean_dt = std::accumulate(deltas.begin(), deltas.end(), 0.0) / deltas.size();
  double variance = 0.0;
  for (double dt : deltas) {
    variance += (dt - mean_dt) * (dt - mean_dt);
  }
  variance /= deltas.size();

  return variance;
}

std::optional<std::tuple<double, double, double, double>> TakeHome::compute_wheel_slip_ratios() {
  if (!latest_odom_) {
    return std::nullopt;  // Need at least odometry
  }

  // Extract odometry data
  double vx = latest_odom_->twist.twist.linear.x;  // m/s
  double vy = latest_odom_->twist.twist.linear.y;  // m/s
  double omega = latest_odom_->twist.twist.angular.z;  // rad/s (yaw rate)

  // Extract wheel speeds (convert kmph to m/s) or use fallback
  double v_rr_w, v_rl_w, v_fr_w, v_fl_w;
  if (latest_wheel_speed_) {
    v_rr_w = latest_wheel_speed_->rear_right / 3.6;
    v_rl_w = latest_wheel_speed_->rear_left / 3.6;
    v_fr_w = latest_wheel_speed_->front_right / 3.6;
    v_fl_w = latest_wheel_speed_->front_left / 3.6;
  } else {
    // Fallback: use vehicle speed as wheel speed (no slip assumption)
    v_rr_w = vx;
    v_rl_w = vx;
    v_fr_w = vx;
    v_fl_w = vx;
  }

  // Extract steering angle and convert to wheel angle, or use fallback
  double delta;
  if (latest_steering_) {
    double steering_deg = latest_steering_->primary_steering_angle_fbk;
    delta = (steering_deg / STEERING_RATIO) * M_PI / 180.0;  // Convert to radians
  } else {
    // Fallback: no steering (straight ahead)
    delta = 0.0;
  }

  // Rear right wheel slip
  double vx_rr = vx - 0.5 * omega * WR;
  double k_rr = 0.0;
  if (std::abs(vx_rr) >= EPSILON) {
    k_rr = (v_rr_w - vx_rr) / vx_rr;
  } else {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                        "Slip denominator too small for RR: vx_rr=%.6f", vx_rr);
  }

  // Rear left wheel slip
  double vx_rl = vx + 0.5 * omega * WR;
  double k_rl = 0.0;
  if (std::abs(vx_rl) >= EPSILON) {
    k_rl = (v_rl_w - vx_rl) / vx_rl;
  } else {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                        "Slip denominator too small for RL: vx_rl=%.6f", vx_rl);
  }

  // Front right wheel slip
  double vx_fr = vx - 0.5 * omega * WF;
  double vy_fr = vy + omega * LF;
  double vx_fr_d = std::cos(delta) * vx_fr - std::sin(delta) * vy_fr;
  double k_fr = 0.0;
  if (std::abs(vx_fr_d) >= EPSILON) {
    k_fr = (v_fr_w - vx_fr_d) / vx_fr_d;
  } else {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                        "Slip denominator too small for FR: vx_fr_d=%.6f", vx_fr_d);
  }

  // Front left wheel slip
  double vx_fl = vx + 0.5 * omega * WF;
  double vy_fl = vy + omega * LF;
  double vx_fl_d = std::cos(delta) * vx_fl - std::sin(delta) * vy_fl;
  double k_fl = 0.0;
  if (std::abs(vx_fl_d) >= EPSILON) {
    k_fl = (v_fl_w - vx_fl_d) / vx_fl_d;
  } else {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                        "Slip denominator too small for FL: vx_fl_d=%.6f", vx_fl_d);
  }

  return std::make_tuple(k_rr, k_rl, k_fr, k_fl);
}

void TakeHome::publish_metrics_timer_callback() {
  // Note: Wheel slip ratios are published in odometry_callback when odometry arrives
  // This timer is for other metrics that need periodic publishing
  
  // Republish slip ratios periodically to ensure topics stay advertised
  // This is a backup in case odometry callback hasn't fired recently
  auto slip_ratios = compute_wheel_slip_ratios();
  if (slip_ratios.has_value()) {
    auto [k_rr, k_rl, k_fr, k_fl] = slip_ratios.value();
    std_msgs::msg::Float32 msg_rr;
    msg_rr.data = static_cast<float>(k_rr);
    slip_rr_publisher_->publish(msg_rr);
    std_msgs::msg::Float32 msg_rl;
    msg_rl.data = static_cast<float>(k_rl);
    slip_rl_publisher_->publish(msg_rl);
    std_msgs::msg::Float32 msg_fr;
    msg_fr.data = static_cast<float>(k_fr);
    slip_fr_publisher_->publish(msg_fr);
    std_msgs::msg::Float32 msg_fl;
    msg_fl.data = static_cast<float>(k_fl);
    slip_fl_publisher_->publish(msg_fl);
  } else {
    // If no odometry yet, publish 0.0 to keep topics advertised
    std_msgs::msg::Float32 init_msg;
    init_msg.data = 0.0;
    slip_rr_publisher_->publish(init_msg);
    slip_rl_publisher_->publish(init_msg);
    slip_fr_publisher_->publish(init_msg);
    slip_fl_publisher_->publish(init_msg);
  }

  // Publish IMU jitter (Task 2B)
  double jitter_top = compute_imu_jitter(imu_top_timestamps_);
  std_msgs::msg::Float32 msg_top;
  msg_top.data = static_cast<float>(jitter_top);
  imu_top_jitter_publisher_->publish(msg_top);

  double jitter_bottom = compute_imu_jitter(imu_bottom_timestamps_);
  std_msgs::msg::Float32 msg_bottom;
  msg_bottom.data = static_cast<float>(jitter_bottom);
  imu_bottom_jitter_publisher_->publish(msg_bottom);

  double jitter_vectornav = compute_imu_jitter(imu_vectornav_timestamps_);
  std_msgs::msg::Float32 msg_vectornav;
  msg_vectornav.data = static_cast<float>(jitter_vectornav);
  imu_vectornav_jitter_publisher_->publish(msg_vectornav);

  // Publish lap time (Task 2C)
  if (is_new_lap_) {
    is_new_lap_ = false;
    std_msgs::msg::Float32 msg_lap;
    msg_lap.data = static_cast<float>(lap_time_);
    lap_time_publisher_->publish(msg_lap);
  }
}

RCLCPP_COMPONENTS_REGISTER_NODE(TakeHome)
