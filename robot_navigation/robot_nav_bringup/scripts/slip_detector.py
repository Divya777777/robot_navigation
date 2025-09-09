#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
import math
import numpy as np

class SlipDetector(Node):
    def __init__(self):
        super().__init__('slip_detector')
        
        # Publishers
        self.slip_factor_pub = self.create_publisher(Float32, 'slip_factor', 10)
        
        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/diff_drive/cmd_vel', self.cmd_vel_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/diff_drive/odom', self.odom_callback, 10)
        
        # Variables
        self.commanded_linear = 0.0
        self.commanded_angular = 0.0
        self.actual_linear = 0.0
        self.actual_angular = 0.0
        self.last_odom_time = None
        self.slip_factor = 0.0
        
        # Parameters
        self.linear_slip_threshold = 0.1  # m/s difference threshold
        self.angular_slip_threshold = 0.2  # rad/s difference threshold
        self.slip_history = []
        self.history_length = 10
        
        # Timer for publishing slip factor
        self.timer = self.create_timer(0.1, self.publish_slip_factor)
        
        self.get_logger().info('Slip Detector Node Started')
    
    def cmd_vel_callback(self, msg):
        self.commanded_linear = abs(msg.linear.x)
        self.commanded_angular = abs(msg.angular.z)
    
    def odom_callback(self, msg):
        # Calculate actual velocities
        self.actual_linear = abs(msg.twist.twist.linear.x)
        self.actual_angular = abs(msg.twist.twist.angular.z)
        
        # Calculate slip
        self.calculate_slip()
    
    def calculate_slip(self):
        # Calculate linear slip
        linear_slip = 0.0
        if self.commanded_linear > 0.05:  # Only check if significant command
            linear_error = abs(self.commanded_linear - self.actual_linear)
            linear_slip = min(linear_error / self.commanded_linear, 1.0)
        
        # Calculate angular slip
        angular_slip = 0.0
        if self.commanded_angular > 0.05:  # Only check if significant command
            angular_error = abs(self.commanded_angular - self.actual_angular)
            angular_slip = min(angular_error / self.commanded_angular, 1.0)
        
        # Combined slip factor
        current_slip = max(linear_slip, angular_slip)
        
        # Add to history and calculate smoothed slip
        self.slip_history.append(current_slip)
        if len(self.slip_history) > self.history_length:
            self.slip_history.pop(0)
        
        # Smoothed slip factor
        self.slip_factor = np.mean(self.slip_history) if self.slip_history else 0.0
        
        # Log high slip events
        if self.slip_factor > 0.3:
            self.get_logger().warn(f'High slip detected: {self.slip_factor:.2f}')
    
    def publish_slip_factor(self):
        msg = Float32()
        msg.data = self.slip_factor
        self.slip_factor_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SlipDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()