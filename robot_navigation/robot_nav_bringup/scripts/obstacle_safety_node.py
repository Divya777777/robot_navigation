#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Twist
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
from collections import deque

class ObstacleSafetyNode(Node):
    def __init__(self):
        super().__init__('obstacle_safety_node')
        
        # Parameters
        self.declare_parameter('min_obstacle_distance', 1.0)  # meters
        self.declare_parameter('safety_distance', 1.5)       # meters
        self.declare_parameter('stop_deceleration', 0.5)     # m/s²
        self.declare_parameter('start_acceleration', 0.3)    # m/s²
        self.declare_parameter('max_speed_reduction', 0.3)   # max speed when obstacle detected
        self.declare_parameter('point_cloud_topic', '/diff_drive/rgbd/points')
        self.declare_parameter('cmd_vel_topic', '/diff_drive/cmd_vel')
        self.declare_parameter('output_cmd_vel_topic', '/cmd_vel_new')
        
        # Get parameters
        self.min_obstacle_distance = self.get_parameter('min_obstacle_distance').value
        self.safety_distance = self.get_parameter('safety_distance').value
        self.stop_deceleration = self.get_parameter('stop_deceleration').value
        self.start_acceleration = self.get_parameter('start_acceleration').value
        self.max_speed_reduction = self.get_parameter('max_speed_reduction').value
        point_cloud_topic = self.get_parameter('point_cloud_topic').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        output_cmd_vel_topic = self.get_parameter('output_cmd_vel_topic').value
        
        # State variables
        self.obstacle_detected = False
        self.min_distance = float('inf')
        self.last_cmd_vel = Twist()
        self.cmd_vel_buffer = deque(maxlen=10)
        self.current_speed = 0.0
        
        # Publishers and Subscribers
        self.point_cloud_sub = self.create_subscription(
            PointCloud2, point_cloud_topic, self.point_cloud_callback, 10)
        
        self.cmd_vel_sub = self.create_subscription(
            Twist, cmd_vel_topic, self.cmd_vel_callback, 10)
        
        self.cmd_vel_pub = self.create_publisher(
            Twist, output_cmd_vel_topic, 10)
        
        # Timer for continuous control
        self.control_timer = self.create_timer(0.1, self.control_callback)  # 10 Hz
        
        self.get_logger().info("Obstacle Safety Node started")
        self.get_logger().info(f"Monitoring point cloud from: {point_cloud_topic}")
        self.get_logger().info(f"Safety distance: {self.safety_distance}m")
    
    def point_cloud_callback(self, msg):
        """Process point cloud data to detect obstacles"""
        try:
            # Read points from the point cloud - handle different field structures
            points = []
            
            # Try to read points with different field combinations
            try:
                # First try with x, y, z fields
                gen = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
                points = list(gen)
            except:
                try:
                    # If that fails, try reading all fields and extract x, y, z
                    gen = pc2.read_points(msg, skip_nans=True)
                    for p in gen:
                        if len(p) >= 3:
                            points.append([p[0], p[1], p[2]])  # x, y, z
                except Exception as e:
                    self.get_logger().warn(f"Could not read point cloud: {e}")
                    return
            
            if len(points) == 0:
                self.min_distance = float('inf')
                self.obstacle_detected = False
                return
            
            # Convert to numpy array
            points_array = np.array(points)
            
            # Calculate distances in front of the robot (ignore z-axis for ground plane)
            # Filter points that are in front of the robot (positive x)
            front_points = points_array[points_array[:, 0] > 0]  # x > 0
            
            if len(front_points) == 0:
                self.min_distance = float('inf')
                self.obstacle_detected = False
                return
            
            # Calculate Euclidean distance (x, y only - ignore height)
            distances = np.sqrt(front_points[:, 0]**2 + front_points[:, 1]**2)
            
            # Find minimum distance
            self.min_distance = np.min(distances)
            
            # Check if obstacle is within safety distance
            self.obstacle_detected = (self.min_distance <= self.safety_distance)
            
            # Log for debugging
            if self.obstacle_detected:
                self.get_logger().debug(
                    f"Obstacle detected at {self.min_distance:.2f}m", 
                    throttle_duration_sec=1.0)
                
        except Exception as e:
            self.get_logger().error(f"Error processing point cloud: {str(e)}")
    
    def cmd_vel_callback(self, msg):
        """Store the latest command velocity"""
        self.cmd_vel_buffer.append(msg)
        if self.cmd_vel_buffer:
            self.last_cmd_vel = self.cmd_vel_buffer[-1]
    
    def control_callback(self):
        """Apply safety control based on obstacle detection"""
        if not self.cmd_vel_buffer:
            return
        
        # Get the latest command velocity
        target_cmd_vel = self.last_cmd_vel
        
        # Create safety-controlled command
        safe_cmd_vel = Twist()
        
        if self.obstacle_detected:
            # Obstacle detected - reduce speed gradually
            safe_cmd_vel = self.apply_obstacle_safety(target_cmd_vel)
        else:
            # No obstacle - gradually return to normal speed
            safe_cmd_vel = self.return_to_normal_speed(target_cmd_vel)
        
        # Update current speed
        self.current_speed = safe_cmd_vel.linear.x
        
        # Publish the safety-controlled command
        self.cmd_vel_pub.publish(safe_cmd_vel)
    
    def apply_obstacle_safety(self, target_cmd_vel):
        """Gradually reduce speed when obstacle is detected"""
        safe_cmd_vel = Twist()
        
        # Calculate speed reduction factor based on distance
        if self.min_distance <= self.min_obstacle_distance:
            # Emergency stop - too close
            safe_cmd_vel.linear.x = 0.0
            safe_cmd_vel.angular.z = 0.0
            
            self.get_logger().warn(
                f"EMERGENCY STOP: Obstacle too close at {self.min_distance:.2f}m")
                
        else:
            # Gradual reduction based on distance
            distance_ratio = (self.min_distance - self.min_obstacle_distance) / \
                            (self.safety_distance - self.min_obstacle_distance)
            reduction_factor = max(0.0, min(1.0, distance_ratio))
            reduction_factor = reduction_factor * (1.0 - self.max_speed_reduction) + self.max_speed_reduction
            
            # Apply reduction with acceleration limits
            target_speed = target_cmd_vel.linear.x * reduction_factor
            max_decel = self.stop_deceleration * 0.1  # 0.1s timer period
            
            safe_cmd_vel.linear.x = max(target_speed, self.current_speed - max_decel)
            
            # Apply more aggressive reduction to angular velocity near obstacles
            angular_reduction = reduction_factor * 0.5  # Reduce turning more
            safe_cmd_vel.angular.z = target_cmd_vel.angular.z * angular_reduction
            
            # Log safety action
            self.get_logger().info(
                f"SAFETY: Obstacle at {self.min_distance:.2f}m, "
                f"Speed reduced to {safe_cmd_vel.linear.x:.2f}m/s",
                throttle_duration_sec=0.5)
        
        return safe_cmd_vel
    
    def return_to_normal_speed(self, target_cmd_vel):
        """Gradually return to normal speed when obstacle is cleared"""
        safe_cmd_vel = Twist()
        
        # Add acceleration limiting for smoother transitions
        max_accel = self.start_acceleration * 0.1  # 0.1s timer period
        
        # Gradually accelerate to target speed
        target_speed = target_cmd_vel.linear.x
        safe_cmd_vel.linear.x = min(self.current_speed + max_accel, target_speed)
        
        # Return angular velocity to normal
        safe_cmd_vel.angular.z = target_cmd_vel.angular.z
        
        # Log return to normal
        if abs(safe_cmd_vel.linear.x - target_speed) < 0.01:
            self.get_logger().info("CLEAR: No obstacles detected, normal speed restored")
        
        return safe_cmd_vel

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleSafetyNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()