#include "take_home_node/take_home.hpp"
#include <rclcpp_components/register_node_macro.hpp>
#include <cmath>


TakeHome::TakeHome(const rclcpp::NodeOptions& options)
    : Node("take_home_metrics", options) {

    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

    Wodometry_subscriber_.subscribe(this, "vehicle/uva_odometry");           // subscribe fro wheel speed calc values
    wspeed_subscriber_.subscribe(this, "raptor_dbw_interface/wheel_speed_report");
    wangle_subscriber_.subscribe(this, "raptor_dbw_interface/steering_extended_report");

    uint32_t queue_size = 10;
    sync = std::make_shared<message_filters::Synchronizer<message_filters::sync_policies::
      ApproximateTime<nav_msgs::msg::Odometry, raptor_dbw_msgs::msg::WheelSpeedReport, raptor_dbw_msgs::msg::SteeringExtendedReport>>>(
      message_filters::sync_policies::
      ApproximateTime<nav_msgs::msg::Odometry, raptor_dbw_msgs::msg::WheelSpeedReport, raptor_dbw_msgs::msg::SteeringExtendedReport>(queue_size),
      Wodometry_subscriber_, wspeed_subscriber_, wangle_subscriber_);

      sync->setAgePenalty(0.50);          // register the callback for wheel speed
      sync->registerCallback(std::bind(&TakeHome::wheelSlip_callback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));


    topImu_subscriber_ = this->create_subscription<novatel_oem7_msgs::msg::RAWIMU>(       // subscribe to imu measurments
      "novatel_top/rawimu", qos_profile,
      std::bind(&TakeHome::topJitter_callback, this, std::placeholders::_1));
    bottomImu_subscriber_ = this->create_subscription<novatel_oem7_msgs::msg::RAWIMU>(
      "novatel_bottom/rawimu", qos_profile,
      std::bind(&TakeHome::botJitter_callback, this, std::placeholders::_1));
    vnImu_subscriber_ = this->create_subscription<vectornav_msgs::msg::CommonGroup>(
      "vectornav/raw/common", qos_profile,
      std::bind(&TakeHome::vnavJitter_callback, this, std::placeholders::_1));

    curvilinear_subscriber_ = this->create_subscription<std_msgs::msg::Float32>(
      "curvilinear_distance", qos_profile,
      std::bind(&TakeHome::laptime_callback, this, std::placeholders::_1));
    
    // Look at the hpp file to define all class variables, including subscribers
    // A subscriber will "listen" to a topic and whenever a message is published to it, the subscriber
    // will pass it onto the attached callback (`TakeHome::odometry_callback` in this instance)
    odometry_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "vehicle/uva_odometry", qos_profile,
      std::bind(&TakeHome::odometry_callback, this, std::placeholders::_1));

    metric_publisher_ = this->create_publisher<std_msgs::msg::Float32>("metrics_output", qos_profile);

    wslip_rr_publisher_ = this->create_publisher<std_msgs::msg::Float32>("slip/long/rr", qos_profile);
    wslip_rl_publisher_ = this->create_publisher<std_msgs::msg::Float32>("slip/long/rl", qos_profile);     // create wheel slip publishers
    wslip_fr_publisher_ = this->create_publisher<std_msgs::msg::Float32>("slip/long/fr", qos_profile);
    wslip_fl_publisher_ = this->create_publisher<std_msgs::msg::Float32>("slip/long/fl", qos_profile);

    jitter_top_publisher_ = this->create_publisher<std_msgs::msg::Float32>("imu_top/jitter", qos_profile);
    jitter_bottom_publisher_ = this->create_publisher<std_msgs::msg::Float32>("imu_bottom/jitter", qos_profile);   // create jitter publishers
    jitter_vn_publisher_ = this->create_publisher<std_msgs::msg::Float32>("imu_vectornav/jitter", qos_profile);

    laptime_publisher_ = this->create_publisher<std_msgs::msg::Float32>("/laptime", rclcpp::QoS(10).reliable());       // create laptime publisher
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

float TakeHome::wheelSlip_rear(float vx, float w, float vw, bool leftW) {       // calculate the wheel slip for rear wheels
  /* vx -> longitudianl linear speed (m/s)
    w -> angular velocity (rad/s)
    vw -> wheel speed (m/s)
    leftW -> true if calc ing for left wheel
  */

  float LorR = leftW ? 1.0f : -1.0f;      
 
  float vx_r = vx + (LorR * 0.5f) * w * WR;   

  if (std::abs(vx_r) < 0.01f) { return 0.0f; }

  float k_r = (vw - vx_r) / vx_r;

  return k_r;
}

float TakeHome::wheelSlip_front(float vx, float w, float vy, float angle, float vw, bool leftW) {    // calculate the wheel slip for front wheels
  /* vx -> longitudianl linear speed (m/s)
    w -> angular velocity (rad/s)
    vy -> lateral linear speed (m/s)
    angle -> steering angle (rad)
    vw -> wheel speed (m/s)
    leftW -> true if calc ing for left wheel
  */

  float LorR = leftW ? 1.0f : -1.0f;

  float vx_f = vx + (LorR * 0.5f) * w * WF;
  float vy_f = vy + w * LF;

  float vx_a_f = std::cos(angle) * vx_f - std::sin(angle) * vy_f;

  if (std::abs(vx_a_f) < 0.01f) { return 0.0f; }

  float k_f = (vw - vx_a_f) / vx_a_f;

  return k_f;
}

float TakeHome::convertSpeed(float kmph) {      // conver from kmph to m/s
  float ms = kmph * (1000.0f / 3600.0f); 
  return ms;
}

float TakeHome::convertAngle(float deg) {     // convert from deg to rad
  float rad = deg * (3.14159f / 180.0f);
  return rad;
}

void TakeHome::wheelSlip_callback(nav_msgs::msg::Odometry::ConstSharedPtr odom_msg,
                        raptor_dbw_msgs::msg::WheelSpeedReport::ConstSharedPtr wheel_msg,
                        raptor_dbw_msgs::msg::SteeringExtendedReport::ConstSharedPtr steer_msg) {
          
  float vx = odom_msg->twist.twist.linear.x;
  float w = odom_msg->twist.twist.angular.z;           // get values for rear wheels slips
  float vw_rr = convertSpeed(wheel_msg->rear_right);
  float vw_rl = convertSpeed(wheel_msg->rear_left);

  float vy = odom_msg->twist.twist.linear.y;
  float angle = convertAngle(steer_msg->primary_steering_angle_fbk / 15.0f);   // get more values for front wheels slips
  float vw_fr = convertSpeed(wheel_msg->front_right);
  float vw_fl = convertSpeed(wheel_msg->front_left);

  //RCLCPP_INFO(this->get_logger(), "syncCallback triggered — messages received");

  std_msgs::msg::Float32  ws_rr;
  std_msgs::msg::Float32  ws_rl;
  std_msgs::msg::Float32  ws_fr;
  std_msgs::msg::Float32  ws_fl;

  ws_rr.data = wheelSlip_rear(vx, w, vw_rr, false);         // get wheel slips for reare wheels
  ws_rl.data = wheelSlip_rear(vx, w, vw_rl, true); 

  ws_fr.data = wheelSlip_front(vx, w, vy, angle, vw_fr, false);   // get wheel slips for front wheels
  ws_fl.data = wheelSlip_front(vx, w, vy, angle, vw_fl, true);

  wslip_rr_publisher_->publish(ws_rr);
  wslip_rl_publisher_->publish(ws_rl);   // publish the wheel slips
  wslip_fr_publisher_->publish(ws_fr);
  wslip_fl_publisher_->publish(ws_fl);
}

float TakeHome::calcJitter(int windowId) {
  auto &window = sWindow[windowId];   // get the window

  if (window.size() < 2) {     // if not enough times to calc deleta t
    return 0.0f;
  }

  std::vector<float> deltaTs;       // create vector to hold delta ts and set size to window - 1
  deltaTs.reserve(window.size() - 1);
  
  for(size_t i = 1; i < window.size(); i++) {
    float deltaT = (window[i] - window[i - 1]).seconds();       // calculate all the delta ts & add to vector
    deltaTs.push_back(deltaT);
  }
  
  float dmean;                       // calcualte the mean of the delta Ts
  for(float deltaT : deltaTs) {
    dmean += deltaT;
  }
  dmean = dmean / deltaTs.size();

  float jitter;
  for(float deltaT: deltaTs) {             // calculate the jitter
    jitter += std::pow(deltaT - dmean, 2);
  }
  jitter = jitter / deltaTs.size();

  return jitter;
}

void TakeHome::updateWindow(int windowId, rclcpp::Time msgTime) {
  auto &window = sWindow[windowId];      // get the Top imu window and add new msg timestamp
  window.push_back(msgTime);                

  rclcpp::Time wboundary = msgTime - rclcpp::Duration::from_seconds(1.0);  // get the begining boundary for the window

  while(!window.empty() && window.front() < wboundary) {    // rremove messages if older than a second
    window.pop_front();
  }
}

void TakeHome::topJitter_callback(novatel_oem7_msgs::msg::RAWIMU::ConstSharedPtr top_msg) {
  rclcpp::Time  msgTime(top_msg->header.stamp);     /// get the time stampe fo the message

  updateWindow(TOP, msgTime);
  float jitter = calcJitter(TOP);

  std_msgs::msg::Float32 jitter_msg;     // publish top jitter
  jitter_msg.data = jitter;
  jitter_top_publisher_->publish(jitter_msg);
}

void TakeHome::botJitter_callback(novatel_oem7_msgs::msg::RAWIMU::ConstSharedPtr bottom_msg) {
  rclcpp::Time  msgTime(bottom_msg->header.stamp);     /// get the time stampe fo the message

  updateWindow(BOTTOM, msgTime);
  float jitter = calcJitter(BOTTOM);

  std_msgs::msg::Float32 jitter_msg;     // publish bottom jitter
  jitter_msg.data = jitter;
  jitter_bottom_publisher_->publish(jitter_msg);
}

void TakeHome::vnavJitter_callback(vectornav_msgs::msg::CommonGroup::ConstSharedPtr vnav_msg) {
  rclcpp::Time  msgTime(vnav_msg->header.stamp);     /// get the time stampe fo the message

  updateWindow(VECTORNAV, msgTime);
  float jitter = calcJitter(VECTORNAV);

  std_msgs::msg::Float32 jitter_msg;     // publish vector nav jitter
  jitter_msg.data = jitter;
  jitter_vn_publisher_->publish(jitter_msg);
}

void TakeHome::laptime_callback(const std_msgs::msg::Float32::SharedPtr msg) {
  rclcpp::Time msgTime = this->now();     // record time of current message
  float dist = msg->data;                        // get curvilinear dist from msg

  if(!lapstartInit && dist < 10) {
    lapStart = std::make_pair(dist, msgTime);   // store first value for when end of lap is reached
    lastMsg = std::make_pair(dist, msgTime);    // also set the last message pair to current message
    lapstartInit = true;
    return;
  }

  float lastdist = lastMsg.first;     // get the dist of the last recived message

  if(dist < lastdist - 10) {                            // see if dist drops significantly w/ a buffer zone fo 10
    rclcpp::Time lstartTime = lapStart.second;                         // get start time
    rclcpp::Duration laptime = msgTime - lstartTime;       // calc the lap time
    float laptime_s = laptime.seconds();

    std_msgs::msg::Float32 laptime_msg;     // publish laptime
    laptime_msg.data = laptime_s;
    laptime_publisher_->publish(laptime_msg);

    lapStart = std::make_pair(dist, msgTime);   // store first value for when end of lap is reached
    lastMsg = std::make_pair(dist, msgTime);    // also set the last message pair to current message
    return;
  }

  lastMsg = std::make_pair(dist, msgTime);    // set the last message pair to the current message
}


RCLCPP_COMPONENTS_REGISTER_NODE(TakeHome)
