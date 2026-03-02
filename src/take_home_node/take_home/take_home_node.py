#!/usr/bin/env python3
"""
Take Home Node - Task 2: Compute metrics and publish on ROS topics
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import math
from collections import deque
from typing import Optional

# Standard ROS2 message types
from std_msgs.msg import Float32, Float64, Header
from nav_msgs.msg import Odometry

# Custom message types from this repo
from raptor_dbw_msgs.msg import WheelSpeedReport, SteeringExtendedReport
from novatel_oem7_msgs.msg import RAWIMU
from vectornav_msgs.msg import CommonGroup


class JitterCalculator:
    """
    Computes jitter (variance of Δt) over a 1-second sliding window.
    Maintains a deque of (sample_time, dt) tuples where sample_time is the message time.
    """
    def __init__(self, name: str, logger):
        self.name = name
        self.logger = logger
        self.last_timestamp: Optional[float] = None
        self.dt_samples: deque = deque()  # (sample_time, dt_seconds) tuples
        self.timestamp_source_logged = False
        self.timestamp_source: Optional[str] = None  # "header" or "node_time"
        self.last_warn_time = {}  # Track last warning time for throttling
    
    def _warn_throttle(self, key: str, message: str, interval: float = 5.0):
        """Simple throttled warning (manual implementation)"""
        import time
        now = time.time()
        if key not in self.last_warn_time or (now - self.last_warn_time[key]) >= interval:
            self.logger.warn(message)
            self.last_warn_time[key] = now
    
    def add_timestamp(self, t_now: float) -> tuple[float, bool]:
        """
        Add a new timestamp and compute jitter.
        Args:
            t_now: Current message timestamp (from header or node time)
        Returns:
            (jitter_variance, is_valid) tuple
        """
        if self.last_timestamp is None:
            self.last_timestamp = t_now
            return (0.0, False)  # Need at least 2 messages
        
        # Compute Δt
        dt = t_now - self.last_timestamp
        
        # Outlier guard: reject dt <= 0 or dt > 0.5
        if dt <= 0:
            self._warn_throttle("dt_negative", f"[{self.name}] Rejected dt <= 0: {dt}")
            return (0.0, False)
        if dt > 0.5:
            self._warn_throttle("dt_large", f"[{self.name}] Rejected dt > 0.5: {dt}")
            return (0.0, False)
        
        # Add (sample_time, dt) to deque
        self.dt_samples.append((t_now, dt))
        
        # Remove samples older than 1.0 second
        while self.dt_samples and (t_now - self.dt_samples[0][0]) > 1.0:
            self.dt_samples.popleft()
        
        # Update last timestamp
        self.last_timestamp = t_now
        
        # Compute variance if we have at least 2 samples
        if len(self.dt_samples) < 2:
            return (0.0, False)
        
        # Extract dt values
        dts = [dt_val for _, dt_val in self.dt_samples]
        
        # Compute population variance
        mean_dt = sum(dts) / len(dts)
        variance = sum((dt_val - mean_dt) ** 2 for dt_val in dts) / len(dts)
        
        return (variance, True)


class TakeHomeNode(Node):
    """
    ROS2 node that computes vehicle metrics:
    - Wheel slip ratios (4 wheels)
    - IMU jitter (3 IMUs)
    - Lap time
    """
    
    def __init__(self):
        super().__init__('take_home_metrics')
        
        # QoS profile for best effort communication
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # ========== SUBSCRIBERS ==========
        # Existing subscription (keep intact)
        self.odometry_subscriber_ = self.create_subscription(
            Odometry,
            'vehicle/uva_odometry',
            self.odometry_callback,
            qos_profile
        )
        
        # New subscriptions for Task 2
        self.wheel_speed_subscriber_ = self.create_subscription(
            WheelSpeedReport,
            'raptor_dbw_interface/wheel_speed_report',
            self.wheel_speed_callback,
            qos_profile
        )
        
        self.steering_subscriber_ = self.create_subscription(
            SteeringExtendedReport,
            'raptor_dbw_interface/steering_extended_report',
            self.steering_callback,
            qos_profile
        )
        
        self.curvilinear_distance_subscriber_ = self.create_subscription(
            Float32,
            'curvilinear_distance',
            self.curvilinear_distance_callback,
            qos_profile
        )
        
        # IMU subscribers
        self.imu_top_subscriber_ = self.create_subscription(
            RAWIMU,
            'novatel_top/rawimu',
            self.imu_top_callback,
            qos_profile
        )
        
        self.imu_bottom_subscriber_ = self.create_subscription(
            RAWIMU,
            'novatel_bottom/rawimu',
            self.imu_bottom_callback,
            qos_profile
        )
        
        self.imu_vectornav_subscriber_ = self.create_subscription(
            CommonGroup,
            'vectornav/raw/common',
            self.imu_vectornav_callback,
            qos_profile
        )
        
        # ========== PUBLISHERS ==========
        # ALL publishers created unconditionally in constructor (NOT in callbacks, NOT behind if-statements)
        # Existing publisher (keep intact) - MUST have leading slash
        self.metric_publisher_ = self.create_publisher(Float32, '/metrics_output', qos_profile)
        
        # Wheel slip ratio publishers (Task 2A) - use Float64 and leading slash
        self.slip_rr_publisher_ = self.create_publisher(Float64, '/slip/long/rr', qos_profile)
        self.slip_rl_publisher_ = self.create_publisher(Float64, '/slip/long/rl', qos_profile)
        self.slip_fr_publisher_ = self.create_publisher(Float64, '/slip/long/fr', qos_profile)
        self.slip_fl_publisher_ = self.create_publisher(Float64, '/slip/long/fl', qos_profile)
        
        # IMU jitter publishers (Task 2B) - use Float64 and leading slashes
        self.imu_top_jitter_publisher_ = self.create_publisher(Float64, '/imu_top/jitter', qos_profile)
        self.imu_bottom_jitter_publisher_ = self.create_publisher(Float64, '/imu_bottom/jitter', qos_profile)
        self.imu_vectornav_jitter_publisher_ = self.create_publisher(Float64, '/imu_vectornav/jitter', qos_profile)
        
        # Lap time publisher (Task 2C) - use Float64 and leading slash
        self.lap_time_publisher_ = self.create_publisher(Float64, '/lap_time', qos_profile)
        
        # ========== STATE VARIABLES ==========
        # Latest received values
        self.latest_odom: Optional[Odometry] = None
        self.latest_wheel_speed: Optional[WheelSpeedReport] = None
        self.latest_steering: Optional[SteeringExtendedReport] = None
        self.latest_curvilinear_distance: Optional[float] = None
        self.prev_curvilinear_distance: Optional[float] = None
        
        # IMU jitter calculators (one per stream)
        self.imu_top_jitter_calc = JitterCalculator("imu_top", self.get_logger())
        self.imu_bottom_jitter_calc = JitterCalculator("imu_bottom", self.get_logger())
        self.imu_vectornav_jitter_calc = JitterCalculator("imu_vectornav", self.get_logger())
        
        # Lap time tracking (Task 2C)
        self.last_s: Optional[float] = None  # Last curvilinear distance (m)
        self.last_t: Optional[float] = None  # Time of last message (seconds)
        self.lap_start_t: Optional[float] = None  # Time when current lap started
        self.lap_count: int = 0
        self.last_published_lap_time: Optional[float] = None
        self.clock_source_logged = False
        self.min_s_observed: Optional[float] = None
        self.max_s_observed: Optional[float] = None
        
        # Lap detection constants
        self.WRAP_THRESHOLD_METERS = 50.0  # Negative ds threshold for wrap detection
        self.HIGH_PROGRESS_MIN = 100.0  # Minimum last_s to consider wrap valid
        self.MIN_LAP_TIME = 1.0  # Minimum reasonable lap time (seconds)
        self.MAX_LAP_TIME = 1000.0  # Maximum reasonable lap time (seconds)
        
        # Constants for wheel slip calculations
        self.WF = 1.638   # Front track width (m)
        self.WR = 1.523   # Rear track width (m)
        self.LF = 1.7238  # CG to front axle distance (m)
        self.EPSILON = 1e-3  # Threshold for division-by-zero checks
        
        # Timer to publish metrics at regular intervals (20 Hz)
        self.timer = self.create_timer(0.05, self.publish_metrics_callback)  # 20 Hz = 0.05s
        
        # Publish initial values immediately to advertise topics (ROS2 topics only appear after first publish)
        # This ensures topics are visible in ros2 topic list even before data arrives
        init_msg_f32 = Float32()
        init_msg_f32.data = 0.0
        init_msg_f64 = Float64()
        init_msg_f64.data = 0.0
        
        self.metric_publisher_.publish(init_msg_f32)  # Advertise /metrics_output
        self.slip_rr_publisher_.publish(init_msg_f64)
        self.slip_rl_publisher_.publish(init_msg_f64)
        self.slip_fr_publisher_.publish(init_msg_f64)
        self.slip_fl_publisher_.publish(init_msg_f64)
        
        # Advertise IMU jitter topics
        self.imu_top_jitter_publisher_.publish(init_msg_f64)
        self.imu_bottom_jitter_publisher_.publish(init_msg_f64)
        self.imu_vectornav_jitter_publisher_.publish(init_msg_f64)
        
        # Advertise lap time topic
        self.lap_time_publisher_.publish(init_msg_f64)
        
        # Log that all publishers are created with exact topic names
        self.get_logger().info(
            'Publishers created: /metrics_output, /slip/long/rr, /slip/long/rl, /slip/long/fr, /slip/long/fl, '
            '/imu_top/jitter, /imu_bottom/jitter, /imu_vectornav/jitter, /lap_time'
        )
        self.get_logger().info('Take Home Node initialized - publishers created and initial messages published')
    
    # ========== CALLBACKS ==========
    
    def odometry_callback(self, msg: Odometry):
        """Existing callback - keep intact with example output"""
        self.latest_odom = msg
        
        # Example metric calculation (keep as-is)
        position_x = msg.pose.pose.position.x
        position_y = msg.pose.pose.position.y
        position_z = msg.pose.pose.position.z
        
        metric_msg = Float32()
        # Avoid division by zero
        denominator = position_x + position_z
        if abs(denominator) < self.EPSILON:
            metric_msg.data = 0.0
        else:
            metric_msg.data = (position_x + position_y + position_z) / denominator
        
        self.metric_publisher_.publish(metric_msg)
    
    def wheel_speed_callback(self, msg: WheelSpeedReport):
        """Store latest wheel speed report"""
        self.latest_wheel_speed = msg
    
    def steering_callback(self, msg: SteeringExtendedReport):
        """Store latest steering extended report"""
        self.latest_steering = msg
    
    def curvilinear_distance_callback(self, msg: Float32):
        """
        Handle curvilinear distance updates and detect lap completion (Task 2C).
        /curvilinear_distance has no header, so we use node clock time.
        """
        # Log clock source once
        if not self.clock_source_logged:
            self.get_logger().info("[lap_time] Using node clock time for /curvilinear_distance (no header)")
            self.clock_source_logged = True
        
        s = msg.data  # Current curvilinear distance (meters)
        t = self.get_clock().now().nanoseconds / 1e9  # Current time (seconds)
        
        # Track min/max for debugging
        if self.min_s_observed is None or s < self.min_s_observed:
            self.min_s_observed = s
        if self.max_s_observed is None or s > self.max_s_observed:
            self.max_s_observed = s
        
        # First message: initialize state
        if self.last_s is None:
            self.last_s = s
            self.last_t = t
            self.lap_start_t = t
            return  # Don't publish yet
        
        # Compute change in distance
        ds = s - self.last_s
        
        # Detect wrap/reset: ds < -WRAP_THRESHOLD and last_s > HIGH_PROGRESS_MIN
        if ds < -self.WRAP_THRESHOLD_METERS and self.last_s > self.HIGH_PROGRESS_MIN:
            # Lap completed!
            if self.lap_start_t is not None:
                lap_time_sec = t - self.lap_start_t
                
                # Validate lap time is reasonable
                if self.MIN_LAP_TIME < lap_time_sec < self.MAX_LAP_TIME:
                    # Publish lap time
                    msg_lap = Float64()
                    msg_lap.data = lap_time_sec
                    self.lap_time_publisher_.publish(msg_lap)
                    
                    self.lap_count += 1
                    self.last_published_lap_time = lap_time_sec
                    
                    # Throttled log
                    self.get_logger().info(
                        f"Lap detected: lap_time={lap_time_sec:.2f}s, "
                        f"lap_count={self.lap_count}, last_s={self.last_s:.2f}m, s={s:.2f}m, "
                        f"ds={ds:.2f}m, s_range=[{self.min_s_observed:.2f}, {self.max_s_observed:.2f}]m"
                    )
                else:
                    self.get_logger().warn(
                        f"Rejected unreasonable lap_time: {lap_time_sec:.2f}s "
                        f"(outside [{self.MIN_LAP_TIME}, {self.MAX_LAP_TIME}]s)"
                    )
            
            # Start new lap
            self.lap_start_t = t
        
        # Update state
        self.last_s = s
        self.last_t = t
        self.latest_curvilinear_distance = s
        self.prev_curvilinear_distance = s
    
    def _extract_timestamp(self, header: Header, calculator: JitterCalculator) -> float:
        """
        Extract timestamp from message header, logging source once per stream.
        Returns timestamp in seconds.
        """
        if header.stamp.sec > 0 or header.stamp.nanosec > 0:
            # Use message timestamp
            if not calculator.timestamp_source_logged:
                calculator.timestamp_source = "header"
                calculator.timestamp_source_logged = True
                self.get_logger().info(f"[{calculator.name}] Using header.stamp for timestamps")
            return header.stamp.sec + header.stamp.nanosec / 1e9
        else:
            # Fallback to node clock
            if not calculator.timestamp_source_logged:
                calculator.timestamp_source = "node_time"
                calculator.timestamp_source_logged = True
                self.get_logger().info(f"[{calculator.name}] Using node time for timestamps (header.stamp not available)")
            return self.get_clock().now().nanoseconds / 1e9
    
    def imu_top_callback(self, msg: RAWIMU):
        """Compute and publish IMU top jitter on each message"""
        t_now = self._extract_timestamp(msg.header, self.imu_top_jitter_calc)
        jitter, is_valid = self.imu_top_jitter_calc.add_timestamp(t_now)
        
        msg_jitter = Float64()
        msg_jitter.data = jitter if is_valid else 0.0
        self.imu_top_jitter_publisher_.publish(msg_jitter)
    
    def imu_bottom_callback(self, msg: RAWIMU):
        """Compute and publish IMU bottom jitter on each message"""
        t_now = self._extract_timestamp(msg.header, self.imu_bottom_jitter_calc)
        jitter, is_valid = self.imu_bottom_jitter_calc.add_timestamp(t_now)
        
        msg_jitter = Float64()
        msg_jitter.data = jitter if is_valid else 0.0
        self.imu_bottom_jitter_publisher_.publish(msg_jitter)
    
    def imu_vectornav_callback(self, msg: CommonGroup):
        """Compute and publish VectorNav IMU jitter on each message"""
        t_now = self._extract_timestamp(msg.header, self.imu_vectornav_jitter_calc)
        jitter, is_valid = self.imu_vectornav_jitter_calc.add_timestamp(t_now)
        
        msg_jitter = Float64()
        msg_jitter.data = jitter if is_valid else 0.0
        self.imu_vectornav_jitter_publisher_.publish(msg_jitter)
    
    # ========== METRIC COMPUTATIONS ==========
    
    def compute_wheel_slip_ratios(self):
        """
        Compute wheel slip ratios for all 4 wheels (Task 2A)
        Returns: (k_rr, k_rl, k_fr, k_fl) or None if inputs missing
        """
        if (self.latest_odom is None or 
            self.latest_wheel_speed is None or 
            self.latest_steering is None):
            return None
        
        # Extract odometry data
        twist = self.latest_odom.twist.twist
        vx = twist.linear.x  # m/s
        vy = twist.linear.y  # m/s
        omega = twist.angular.z  # rad/s (yaw rate)
        
        # Extract wheel speeds (convert kmph to m/s)
        v_rr_w = self.latest_wheel_speed.rear_right / 3.6
        v_rl_w = self.latest_wheel_speed.rear_left / 3.6
        v_fr_w = self.latest_wheel_speed.front_right / 3.6
        v_fl_w = self.latest_wheel_speed.front_left / 3.6
        
        # Extract steering angle and convert to wheel angle
        # primary_steering_angle_fbk is in degrees, convert to wheel angle delta (rad)
        steering_deg = self.latest_steering.primary_steering_angle_fbk
        delta = math.radians(steering_deg / 15.0)  # Divide by steering ratio 15.0
        
        # Rear right wheel slip
        vx_rr = vx - 0.5 * omega * self.WR
        if abs(vx_rr) < self.EPSILON:
            k_rr = 0.0
        else:
            k_rr = (v_rr_w - vx_rr) / vx_rr
        
        # Rear left wheel slip
        vx_rl = vx + 0.5 * omega * self.WR
        if abs(vx_rl) < self.EPSILON:
            k_rl = 0.0
        else:
            k_rl = (v_rl_w - vx_rl) / vx_rl
        
        # Front right wheel slip
        vx_fr = vx - 0.5 * omega * self.WF
        vy_fr = vy + omega * self.LF
        vx_fr_d = math.cos(delta) * vx_fr - math.sin(delta) * vy_fr
        if abs(vx_fr_d) < self.EPSILON:
            k_fr = 0.0
        else:
            k_fr = (v_fr_w - vx_fr_d) / vx_fr_d
        
        # Front left wheel slip
        vx_fl = vx + 0.5 * omega * self.WF
        vy_fl = vy + omega * self.LF
        vx_fl_d = math.cos(delta) * vx_fl - math.sin(delta) * vy_fl
        if abs(vx_fl_d) < self.EPSILON:
            k_fl = 0.0
        else:
            k_fl = (v_fl_w - vx_fl_d) / vx_fl_d
        
        return (k_rr, k_rl, k_fr, k_fl)
    
    
    
    # ========== PUBLISHING ==========
    
    def publish_metrics_callback(self):
        """Timer callback to publish all metrics"""
        
        # Publish wheel slip ratios (Task 2A)
        slip_ratios = self.compute_wheel_slip_ratios()
        if slip_ratios is not None:
            k_rr, k_rl, k_fr, k_fl = slip_ratios
            
            msg_rr = Float64()
            msg_rr.data = float(k_rr)
            self.slip_rr_publisher_.publish(msg_rr)
            
            msg_rl = Float64()
            msg_rl.data = float(k_rl)
            self.slip_rl_publisher_.publish(msg_rl)
            
            msg_fr = Float64()
            msg_fr.data = float(k_fr)
            self.slip_fr_publisher_.publish(msg_fr)
            
            msg_fl = Float64()
            msg_fl.data = float(k_fl)
            self.slip_fl_publisher_.publish(msg_fl)
        
        # Note: IMU jitter is published directly in callbacks (Task 2B)
        # Note: Lap time is published directly in curvilinear_distance_callback when lap is detected (Task 2C)
        # No need to publish here as they're done on message arrival


def main(args=None):
    rclpy.init(args=args)
    node = TakeHomeNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
