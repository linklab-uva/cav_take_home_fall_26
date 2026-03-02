#include "take_home_node/take_home.hpp"
#include <rclcpp_components/register_node_macro.hpp>
#include <cmath>
#include <deque>

//A:
//basically the trick part is that we need to write more subscribers and callback
//  functions to get wheel velocities and the steering angle. We also have to do conversions
// I think the best setup is subscribing to each topic with its own callback method  and then just doing the rest
//in odemetry callback since its gaurenteed calledback and the comments are kind of hinting to write there.
// ( // Do stuff with this callback! or more, idc)


//B:
//computers the jitter of each imu by subscribing to the data and reading the header which has the time stamped data
//then I made a helper function computer jitter to calculate the difference between the input times (it should be consistent ideally). We keep tje time stamps to make a 1 second sliding window like asked, and there are three topics which publish that difference.


// C:
// computes lap time by subscribing to the curvilinear distance topic which increases as the car moves along the track
// when the distance resets back to a smaller value, it means a lap was completed
// I record the time using the message timestamp and subtract the previous lap time to get the lap duration
// then I publish the lap time to a new topic so it can be plotted and visualized in foxglove

//In general I just tried to make separate publishers and subscirbers for one data value at a time. 

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

    wheel_speed_subscriber_ = this->create_subscription<raptor_dbw_msgs::msg::WheelSpeedReport>(
        "raptor_dbw_interface/wheel_speed_report", qos_profile,
        std::bind(&TakeHome::wheel_speed_callback, this, std::placeholders::_1));

    steering_subscriber_ = this->create_subscription<raptor_dbw_msgs::msg::SteeringExtendedReport>(
        "raptor_dbw_interface/steering_extended_report", qos_profile,
        std::bind(&TakeHome::steering_callback, this, std::placeholders::_1));

    curvilinear_subscriber_ = this->create_subscription<std_msgs::msg::Float32>(
      "curvilinear_distance", qos_profile,
      std::bind(&TakeHome::curvilinear_callback, this, std::placeholders::_1));

    lap_time_publisher_ = this->create_publisher<std_msgs::msg::Float32>(
        "lap_time", qos_profile);

    slip_rr_publisher_ = this->create_publisher<std_msgs::msg::Float32>("slip/long/rr", qos_profile);
    slip_rl_publisher_ = this->create_publisher<std_msgs::msg::Float32>("slip/long/rl", qos_profile);
    slip_fr_publisher_ = this->create_publisher<std_msgs::msg::Float32>("slip/long/fr", qos_profile);
    slip_fl_publisher_ = this->create_publisher<std_msgs::msg::Float32>("slip/long/fl", qos_profile);




    //part b:
    imu_top_jitter_pub_ = this->create_publisher<std_msgs::msg::Float32>("imu_top/jitter", qos_profile);

    imu_bottom_jitter_pub_ = this->create_publisher<std_msgs::msg::Float32>("imu_bottom/jitter", qos_profile);

    imu_vectornav_jitter_pub_ = this->create_publisher<std_msgs::msg::Float32>("imu_vectornav/jitter", qos_profile);




    imu_top_subscriber_ = this->create_subscription<novatel_oem7_msgs::msg::RAWIMU>(
        "novatel_top/rawimu", qos_profile,
        std::bind(&TakeHome::imu_top_callback, this, std::placeholders::_1));
    

    imu_bottom_subscriber_ = this->create_subscription<novatel_oem7_msgs::msg::RAWIMU>(
        "novatel_bottom/rawimu", qos_profile,
        std::bind(&TakeHome::imu_bottom_callback, this, std::placeholders::_1));
    

    imu_vectornav_subscriber_ = this->create_subscription<vectornav_msgs::msg::CommonGroup>(
        "vectornav/raw/common", qos_profile,
        std::bind(&TakeHome::imu_vectornav_callback, this, std::placeholders::_1));
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


  //now that we have all the values from the 3 callback functions we can actually compute and pubish the 4 equations for the slip of each wheel 
  //given values:

  float wr = 1.523;
  float wf = 1.638;
  float lf = 1.7238;

  // odometry values:

  float vx = odom_msg->twist.twist.linear.x;
  float vy = odom_msg->twist.twist.linear.y;
  float omega = odom_msg->twist.twist.angular.z;

  // wheel speed value and unit conversion:

  float v_rr_w = rear_right_ / 3.6;
  float v_rl_w = rear_left_  / 3.6;
  float v_fr_w = front_right_ / 3.6;
  float v_fl_w = front_left_  / 3.6;

  //steeringn angle deg, must be computed to radians
  float delta = (steering_motor_angle_deg_ / 15.0f) * (M_PI / 180.0f);

  //krr equation
  float vx_rr = vx - 0.5f * omega * wr;
  float krr = (v_rr_w - vx_rr) / vx_rr;

  //kll equation 
  float vx_rl = vx + 0.5f * omega * wr;
  float krl = (v_rl_w - vx_rl) / vx_rl;

  //kfr equation
  float vx_fr = vx - 0.5f * omega * wf;
  float vy_fr = vy + omega * lf;
  float vx_fr_delta = std::cos(delta) * vx_fr - std::sin(delta) * vy_fr;
  float kfr = (v_fr_w - vx_fr_delta) / vx_fr_delta;

  //kfl equation

  float vx_fl = vx + 0.5f * omega * wf;
  float vy_fl = vy + omega * lf;
  float vx_fl_delta = std::cos(delta) * vx_fl - std::sin(delta) * vy_fl;
  float kfl = (v_fl_w - vx_fl_delta) / vx_fl_delta;

  std_msgs::msg::Float32 msg_rr;
  msg_rr.data = krr;
  slip_rr_publisher_->publish(msg_rr);

  std_msgs::msg::Float32 msg_rl;
  msg_rl.data = krl;
  slip_rl_publisher_->publish(msg_rl);

  std_msgs::msg::Float32 msg_fr;
  msg_fr.data = kfr;
  slip_fr_publisher_->publish(msg_fr);

  std_msgs::msg::Float32 msg_fl;
  msg_fl.data = kfl;
  slip_fl_publisher_->publish(msg_fl);


}


void TakeHome::wheel_speed_callback(
  raptor_dbw_msgs::msg::WheelSpeedReport::ConstSharedPtr msg)
{
    rear_right_ = msg->rear_right;
    rear_left_ = msg->rear_left;
    front_right_ = msg->front_right;
    front_left_ = msg->front_left;
}

void TakeHome::steering_callback(
  raptor_dbw_msgs::msg::SteeringExtendedReport::ConstSharedPtr msg)
{
    steering_motor_angle_deg_ = msg->primary_steering_angle_fbk;
}

void TakeHome::imu_vectornav_callback(
    vectornav_msgs::msg::CommonGroup::ConstSharedPtr msg)
{
    double time =
        msg->header.stamp.sec +
        msg->header.stamp.nanosec * 1e-9;

    float jitter = compute_jitter(imu_vectornav_times_, time);

    std_msgs::msg::Float32 out;
    out.data = jitter;

    imu_vectornav_jitter_pub_->publish(out);
}

void TakeHome::imu_top_callback(
    novatel_oem7_msgs::msg::RAWIMU::ConstSharedPtr msg)
{
    double time =
        msg->header.stamp.sec +
        msg->header.stamp.nanosec * 1e-9;

    float jitter = compute_jitter(imu_top_times_, time);

    std_msgs::msg::Float32 out;
    out.data = jitter;

    imu_top_jitter_pub_->publish(out);
}

void TakeHome::imu_bottom_callback(
    novatel_oem7_msgs::msg::RAWIMU::ConstSharedPtr msg)
{
    double time =
        msg->header.stamp.sec +
        msg->header.stamp.nanosec * 1e-9;

    float jitter = compute_jitter(imu_bottom_times_, time);

    std_msgs::msg::Float32 out;
    out.data = jitter;

    imu_bottom_jitter_pub_->publish(out);
}

float TakeHome::compute_jitter(std::deque<double>& times, double current_time)
{
    times.push_back(current_time);

    while (!times.empty() && times.front() < current_time - 1.0)
        times.pop_front();

    if (times.size() < 2)
        return 0.0f;

    double sum = 0.0;
    for (size_t i = 1; i < times.size(); i++)
        sum += times[i] - times[i-1];

    double mean = sum / (times.size() - 1);

    double variance = 0.0;
    for (size_t i = 1; i < times.size(); i++)
    {
        double dt = times[i] - times[i-1];
        variance += (dt - mean) * (dt - mean);
    }

    variance /= (times.size() - 1);

    return (float)variance;
}


void TakeHome::curvilinear_callback(
    std_msgs::msg::Float32::ConstSharedPtr msg)
{
    float current_distance = msg->data;

    double current_time =
        this->get_clock()->now().seconds();

    if (!first_distance_received_)
    {
        previous_distance_ = current_distance;
        last_lap_time_ = current_time;
        first_distance_received_ = true;
        return;
    }

    if (current_distance < previous_distance_)
    {
        double lap_time = current_time - last_lap_time_;

        std_msgs::msg::Float32 out;
        out.data = lap_time;

        lap_time_publisher_->publish(out);

        last_lap_time_ = current_time;
    }

    previous_distance_ = current_distance;
}

RCLCPP_COMPONENTS_REGISTER_NODE(TakeHome)