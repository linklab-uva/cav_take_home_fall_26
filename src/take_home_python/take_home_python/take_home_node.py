#!/usr/bin/env python3
'''
----- Python template for a publisher -------- 

# In __init__:
self.my_publisher = self.create_publisher(
    MessageType,        # Type of message (Float32, String, etc.)
    'topic_name',       # Name of the topic
    10)                 # Queue size


----- Python template for a subscriber -------

self.my_subscriber = self.create_subscription(
    MessageType,           # Type of message
    'topic_name',          # Name of the topic
    self.my_callback,      # Function to call when message arrives
    10)                    # Queue size

# Define the callback function:
def my_callback(self, msg):
    value = msg.data       # Extract data from message
    # perform an action with the value

'''


import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32            # message type is Float32
from nav_msgs.msg import Odometry
from raptor_dbw_msgs.msg import WheelSpeedReport, SteeringExtendedReport
from novatel_oem7_msgs.msg import RAWIMU        # message type for all topics with novatel
from vectornav_msgs.msg import CommonGroup      # message type for all topics with vectornav
import math
from collections import deque

class TakeHomeNode(Node):
    def __init__(self):
        super().__init__('take_home_metrics')
        
        # qoS profile
        qos_profile = rclpy.qos.QoSProfile(
            reliability = rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            history = rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth = 10
        )
        
        #  implement task c
        self.curvilinear_sub = self.create_subscription(            # subscriber for curvliniear distance
            Float32,
            'curvilinear_distance',
            self.curvilinear_callback,
            qos_profile)
        
        self.lap_time_pub = self.create_publisher(Float32, 'lap_time', qos_profile)     # publisher for lap time
        
        # Variables for lap tracking
        self.last_curvilinear_distance = 0.0
        self.lap_start_time = self.get_clock().now()
        self.lap_in_progress = False

    
       # task a: subscriber for vehicle/uva_odometry
        self.odom_sub = self.create_subscription(        
            Odometry,
            'vehicle/uva_odometry',
            self.odometry_callback,
            qos_profile)
        
        #  subscriber for wheel_speed_report
        self.wheel_speed_sub = self.create_subscription(
            WheelSpeedReport,
            'raptor_dbw_interface/wheel_speed_report',
            self.wheel_speed_callback,
            qos_profile)
 
        # subscriber for steering_extended_report
        self.steering_sub = self.create_subscription(
            SteeringExtendedReport,
            'raptor_dbw_interface/steering_extended_report',
            self.steering_callback,
            qos_profile)
        
        # publishers for 4 wheel slip topics
        self.slip_rr_publisher = self.create_publisher(Float32, 'slip/long/rr', qos_profile)      # publisher for rear right wheel
        self.slip_rl_publisher = self.create_publisher(Float32, 'slip/long/rl', qos_profile)      # publisher for rear left wheel
        self.slip_fr_publisher = self.create_publisher(Float32, 'slip/long/fr', qos_profile)      # publisher for front right wheel
        self.slip_fl_publisher = self.create_publisher(Float32, 'slip/long/fl', qos_profile)      # publisher for front left wheel

        # initialize variables for wheel slip calculation

        self.vx = 0.0                           # longitudinal velocity (m/s)
        self.vy = 0.0                           # lateral velocity (m/s)
        self.omega = 0.0                        # yaw rate (rad/s)
        self.wheel_speeds = {
            'fl': 0.0, 
            'fr': 0.0,                          # unit (km/h)
            'rl': 0.0, 
            'rr': 0.0}  
        self.steering_angle = 0.0               # degrees


        # constants
        self.WF = 1.638             # front track width (m)
        self.WR = 1.523             # rear track width (m)
        self.LF = 1.7238            # distance from COG to front wheels (m)
       
        
        # task b:  create subscribers for imu topics
        self.top_imu_sub = self.create_subscription(
            RAWIMU,
            'novatel_top/rawimu',
            self.top_imu_callback,
            qos_profile
        )

        self.bottom_imu_sub = self.create_subscription(
            RAWIMU,
            'novatel_bottom/rawimu',
            self.bottom_imu_callback,
            qos_profile
        )

        self.vectornav_imu_sub = self.create_subscription(
            CommonGroup,
            'vectornav/raw/common',
            self.vectornav_imu_callback,
            qos_profile 
        )

        # Create publishers for 3 jitter topics

        self.top_jitter_pub = self.create_publisher(Float32, 'imu_top/jitter', qos_profile)         # publsiher for top jitter
        self.bottom_jitter_pub = self.create_publisher(Float32, 'imu_bottom/jitter', qos_profile)   # publisher for bottom jitter
        self.vectornav_jitter_pub = self.create_publisher(Float32, 'imu_vectornav/jitter', qos_profile)     # publisher for 

        # deques for timestamp tracking ( a python deque can be used to store this effectively for a sliding window approach)
        self.top_imu_timestamps = deque(maxlen=200)
        self.bottom_imu_timestamps = deque(maxlen=200)
        self.vectornav_imu_timestamps = deque(maxlen=200)


        self.get_logger().info('Timestamp tracking started...')
    
    #  TASK C: LAP TIME CALLBACKS 
    def curvilinear_callback(self, msg):
        current_distance = msg.data
        
        # Finding when the lap is completed
        if self.lap_in_progress and self.last_curvilinear_distance > 100.0 and current_distance < 50.0:

            lap_end_time = self.get_clock().now()       # we've completed a lap (store the current time)
            lap_time = (lap_end_time - self.lap_start_time).nanoseconds/1e9       # in seconds 
            
            # Publish lap time
            lap_msg = Float32()
            lap_msg.data = lap_time
            self.lap_time_pub.publish(lap_msg)          # publish the message to lap_time
            
            self.get_logger().info(f'A lap has been completed. Time taken: {lap_time:.2f} seconds')
            
            # Reset for next lap
            self.lap_start_time = lap_end_time          # set the start time equal to the end of the last time the last lap took
        
        # If lap is in progress, and we've gone a distance of 10 or more, start tracking time.  
        if not self.lap_in_progress and current_distance > 10.0:
            self.lap_in_progress = True
            self.lap_start_time = self.get_clock().now()
        
        # Remember this distance for next callback
        self.last_curvilinear_distance = current_distance
    
    #  TASK A: WHEEL SLIP CALLBACKS 
    def odometry_callback(self, msg):
        self.vx = msg.twist.twist.linear.x   # longitudinal linear speed
        self.vy = msg.twist.twist.linear.y   # sideways velocity
        self.omega = msg.twist.twist.angular.z  # angular velocity (yaw rate)
    
    def wheel_speed_callback(self, msg):
        # Store wheel speeds 
        self.wheel_speeds['fl'] = msg.front_left * (1000/3600)  # convert to m/s
        self.wheel_speeds['fr'] = msg.front_right * (1000/3600) 
        self.wheel_speeds['rl'] = msg.rear_left * (1000/3600)  
        self.wheel_speeds['rr'] = msg.rear_right * (1000/3600)  
        
        self.calculate_wheel_slip()     # calculate and publish wheel slip
    
    def steering_callback(self, msg):
        # Extract steering angle and convert to radians
        motor_rot_angle = msg.primary_steering_angle_fbk                # degrees from motor
        wheel_angle =  motor_rot_angle / 15.0                   # divide steering degrees by the steering ratio to get the wheel angle
        self.steering_angle = math.radians(wheel_angle)         # convert to radians (metric)


    # formulas for calculating whell slip
    def calculate_wheel_slip(self):

        # rear right wheel
        vx_rr = self.vx - 0.5 * self.omega * self.WR

        if abs(vx_rr) < 0.1:    
            kappa_rr = 0.0      # Assume slip is zero at low speed
        else:
            kappa_rr = (self.wheel_speeds['rr'] - vx_rr) / vx_rr

        rr_msg = Float32()
        rr_msg.data = float(kappa_rr)
        self.slip_rr_publisher.publish(rr_msg)
        
        # rear left wheel 
        vx_rl = self.vx + 0.5 * self.omega * self.WR

        if abs(vx_rl) < 0.1:
            kappa_rl = 0.0
        else: 
            kappa_rl = (self.wheel_speeds['rl']- vx_rl) / vx_rl
            
        rl_msg = Float32()
        rl_msg.data = float(kappa_rl)
        self.slip_rl_publisher.publish(rl_msg)

        # front right wheel
        vx_fr = self.vx - 0.5 * self.omega * self.WF
        vy_fr = self.vy + self.omega * self.LF
        vx_fr_delta = math.cos(self.steering_angle) * vx_fr - math.sin(self.steering_angle) * vy_fr
        
        if abs(vx_fr_delta) < 0.1:   
            kappa_fr = 0.0
        else:
            kappa_fr = (self.wheel_speeds['fr'] - vx_fr_delta) / vx_fr_delta
        
        fr_msg = Float32()
        fr_msg.data = float(kappa_fr)
        self.slip_fr_publisher.publish(fr_msg)

        # front left wheel
        vx_fl = self.vx + 0.5 * self.omega * self.WF
        vy_fl = self.vy + self.omega * self.LF
        vx_fl_delta = math.cos(self.steering_angle) * vx_fl - math.sin(self.steering_angle) * vy_fl
        
        if abs(vx_fl_delta) < 0.1:   
            kappa_fl = 0.0
        else:
            kappa_fl = (self.wheel_speeds['fl'] - vx_fl_delta) / vx_fl_delta
        
        fl_msg = Float32()
        fl_msg.data = float(kappa_fl)
        self.slip_fl_publisher.publish(fl_msg)



    #  task b: imu jitter callbacks 
    def top_imu_callback(self, msg):
    
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9       # Get timestamps
    
        self.top_imu_timestamps.append(timestamp)        # Add to sliding window
        
        
        if len(self.top_imu_timestamps) >= 2:           # if we have enough data to calcuate the difference (deltas)

            delta_t = []
            for i in range(len(self.top_imu_timestamps) - 1):
                dt = self.top_imu_timestamps[i+1] - self.top_imu_timestamps[i]
                delta_t.append(dt)
            
            if len(delta_t) > 0:
                mean_dt = sum(delta_t) / len(delta_t)       # calcuate mean after you have some data 
                
                # Calculate variance (jitter)
                variance = sum((dt - mean_dt)**2 for dt in delta_t) / len(delta_t)
                
                # Publish
                jitter_msg = Float32()
                jitter_msg.data = float(variance)
                self.top_jitter_pub.publish(jitter_msg)


    # bottom imu callback
    def bottom_imu_callback(self, msg):
    
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9       # Get timestamps
    
        self.bottom_imu_timestamps.append(timestamp)        # Add to sliding window
        
        
        if len(self.bottom_imu_timestamps) >= 2:           # if we have enough data to calcuate the difference (deltas)

            delta_t = []
            for i in range(len(self.bottom_imu_timestamps) - 1):
                dt = self.bottom_imu_timestamps[i+1] - self.bottom_imu_timestamps[i]
                delta_t.append(dt)
            
            if len(delta_t) > 0:
                mean_dt = sum(delta_t) / len(delta_t)       # calcuate mean after you have some data 
                
                # Calculate variance (jitter)
                variance = sum((dt - mean_dt)**2 for dt in delta_t) / len(delta_t)
                
                # Publish
                jitter_msg = Float32()
                jitter_msg.data = float(variance)
                self.bottom_jitter_pub.publish(jitter_msg)
    
    # vectornav imu callback
    def vectornav_imu_callback(self, msg):
    
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9       # Get timestamps
    
        self.vectornav_imu_timestamps.append(timestamp)        # Add to sliding window
        
        
        if len(self.vectornav_imu_timestamps) >= 2:           # if we have enough data to calcuate the difference (deltas)

            delta_t = []
            for i in range(len(self.vectornav_imu_timestamps) - 1):
                dt = self.vectornav_imu_timestamps[i+1] - self.vectornav_imu_timestamps[i]
                delta_t.append(dt)
            
            if len(delta_t) > 0:
                mean_dt = sum(delta_t) / len(delta_t)       # calcuate mean after you have some data 
                
                # Calculate variance (jitter)
                variance = sum((dt - mean_dt)**2 for dt in delta_t) / len(delta_t)
                
                # Publish
                jitter_msg = Float32()
                jitter_msg.data = float(variance)
                self.vectornav_jitter_pub.publish(jitter_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TakeHomeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()