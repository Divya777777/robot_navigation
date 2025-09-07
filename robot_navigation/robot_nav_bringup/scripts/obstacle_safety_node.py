#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

class ObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance_node')
        
        # Initialize parameters
        self.current_cmd_vel = Twist()
        self.modified_cmd_vel = Twist()
        
        # Declare and get parameters
        self.declare_parameter('obstacle_threshold', 1.0)
        self.declare_parameter('safety_zone_width', 0.3)
        self.declare_parameter('safety_zone_height', 0.5)
        self.declare_parameter('max_deceleration_rate', 0.2)
        self.declare_parameter('max_acceleration_rate', 0.05)
        self.declare_parameter('min_speed_factor', 0.0)
        self.declare_parameter('depth_topic', '/diff_drive/depth/image_raw')
        self.declare_parameter('input_cmd_vel_topic', '/diff_drive/cmd_vel')
        self.declare_parameter('output_cmd_vel_topic', '/cmd_vel_new')
        self.declare_parameter('debug_mode', True)
        
        # Get parameter values
        self.obstacle_threshold = self.get_parameter('obstacle_threshold').get_parameter_value().double_value
        self.safety_zone_width = self.get_parameter('safety_zone_width').get_parameter_value().double_value
        self.safety_zone_height = self.get_parameter('safety_zone_height').get_parameter_value().double_value
        self.max_deceleration_rate = self.get_parameter('max_deceleration_rate').get_parameter_value().double_value
        self.max_acceleration_rate = self.get_parameter('max_acceleration_rate').get_parameter_value().double_value
        self.min_speed_factor = self.get_parameter('min_speed_factor').get_parameter_value().double_value
        self.debug_mode = self.get_parameter('debug_mode').get_parameter_value().bool_value
        
        # Topic names
        depth_topic = self.get_parameter('depth_topic').get_parameter_value().string_value
        input_cmd_vel_topic = self.get_parameter('input_cmd_vel_topic').get_parameter_value().string_value
        output_cmd_vel_topic = self.get_parameter('output_cmd_vel_topic').get_parameter_value().string_value
        
        # State variables
        self.obstacle_detected = False
        self.current_speed_factor = 1.0
        self.last_obstacle_time = self.get_clock().now()
        self.last_log_time = self.get_clock().now()
        self.depth_received = False
        self.cmd_vel_received = False
        self.min_distance = float('inf')
        
        # QoS Profiles
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Create subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            input_cmd_vel_topic,
            self.cmd_vel_callback,
            reliable_qos
        )
        
        self.depth_sub = self.create_subscription(
            Image,
            depth_topic,
            self.depth_callback,
            sensor_qos
        )
        
        # Create publisher
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            output_cmd_vel_topic,
            reliable_qos
        )
        
        # Create timer for publishing modified velocities
        self.publish_timer = self.create_timer(0.05, self.publish_modified_velocity)  # 20Hz
        
        # Debug timer
        if self.debug_mode:
            self.debug_timer = self.create_timer(1.0, self.debug_status)
        
        self.get_logger().info("=== Obstacle Avoidance Node Initialized ===")
        self.get_logger().info(f"Obstacle threshold: {self.obstacle_threshold}m")
        self.get_logger().info(f"Safety zone: {self.safety_zone_width} x {self.safety_zone_height}")
        self.get_logger().info(f"Subscribed to depth: {depth_topic}")
        self.get_logger().info(f"Subscribed to cmd_vel: {input_cmd_vel_topic}")
        self.get_logger().info(f"Publishing to: {output_cmd_vel_topic}")
        self.get_logger().info(f"Debug mode: {self.debug_mode}")
        self.get_logger().info("=======================================")
    
    def discover_depth_topic(self):
        """Auto-discover depth topics"""
        import time
        time.sleep(1)  # Wait for topic discovery
        
        topic_names = self.get_topic_names_and_types()
        depth_topics = []
        
        for topic_name, topic_types in topic_names:
            if 'sensor_msgs/msg/Image' in topic_types and 'depth' in topic_name.lower():
                depth_topics.append(topic_name)
        
        if depth_topics:
            self.get_logger().info(f"Found depth topics: {depth_topics}")
            return depth_topics[0]  # Use first found
        else:
            self.get_logger().warn("No depth topics found!")
            return None
    
    def create_depth_subscriber(self, depth_topic):
        """Create depth subscriber with fallback QoS profiles"""
        qos_profiles = [
            # Try BEST_EFFORT first (common for sensors)
            QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=1
            ),
            # Fallback to RELIABLE
            QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
                depth=1
            ),
            # Try with different durability
            QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                durability=DurabilityPolicy.VOLATILE
            )
        ]
        
        for i, qos in enumerate(qos_profiles):
            try:
                self.get_logger().info(f"Trying to subscribe to {depth_topic} with QoS profile {i+1}")
                self.depth_sub = self.create_subscription(
                    Image,
                    depth_topic,
                    self.depth_callback,
                    qos
                )
                self.get_logger().info(f"Successfully subscribed to {depth_topic}")
                break
            except Exception as e:
                self.get_logger().warn(f"QoS profile {i+1} failed: {e}")
                continue
    
    def debug_status(self):
        """Debug function to print status"""
        self.get_logger().info(f"=== DEBUG STATUS ===")
        self.get_logger().info(f"Depth received: {self.depth_received}")
        self.get_logger().info(f"Cmd_vel received: {self.cmd_vel_received}")
        self.get_logger().info(f"Obstacle detected: {self.obstacle_detected}")
        self.get_logger().info(f"Min distance: {self.min_distance:.2f}m")
        self.get_logger().info(f"Speed factor: {self.current_speed_factor:.2f}")
        self.get_logger().info(f"Input cmd_vel - linear.x: {self.current_cmd_vel.linear.x:.2f}")
        self.get_logger().info(f"Output cmd_vel - linear.x: {self.modified_cmd_vel.linear.x:.2f}")
        self.get_logger().info("==================")
    
    def cmd_vel_callback(self, msg):
        """Store the incoming velocity commands"""
        self.current_cmd_vel = msg
        self.cmd_vel_received = True
        if self.debug_mode and abs(msg.linear.x) > 0.01:
            self.get_logger().info(f"Received cmd_vel: linear.x={msg.linear.x:.2f}, angular.z={msg.angular.z:.2f}")
    
    def image_to_numpy(self, img_msg):
        """Convert ROS2 Image message to numpy array without cv_bridge"""
        try:
            height = img_msg.height
            width = img_msg.width
            encoding = img_msg.encoding
            
            if self.debug_mode:
                self.get_logger().info(f"Image: {width}x{height}, encoding: {encoding}")
            
            # Convert image data to numpy array
            if encoding in ["16UC1", "mono16"]:
                dtype = np.uint16
                depth_image = np.frombuffer(img_msg.data, dtype=dtype).reshape(height, width)
                # Convert to meters (assuming input is in millimeters)
                depth_image = depth_image.astype(np.float32) / 1000.0
            elif encoding == "32FC1":
                dtype = np.float32
                depth_image = np.frombuffer(img_msg.data, dtype=dtype).reshape(height, width)
            else:
                # Try to handle other encodings
                self.get_logger().warn(f"Unknown encoding: {encoding}, trying as 16-bit")
                dtype = np.uint16
                try:
                    depth_image = np.frombuffer(img_msg.data, dtype=dtype).reshape(height, width)
                    depth_image = depth_image.astype(np.float32) / 1000.0
                except:
                    self.get_logger().error(f"Failed to decode image with encoding {encoding}")
                    return None
            
            return depth_image
            
        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")
            return None
    
    def depth_callback(self, msg):
        """Process depth image and detect obstacles"""
        self.depth_received = True
        
        try:
            # Convert ROS2 Image message to numpy array
            depth_image = self.image_to_numpy(msg)
            
            if depth_image is None:
                return
            
            # Get image dimensions
            height, width = depth_image.shape
            
            # Define safety zone (center portion of the image)
            zone_width = int(width * self.safety_zone_width)
            zone_height = int(height * self.safety_zone_height)
            
            start_x = max(0, (width - zone_width) // 2)
            end_x = min(width, start_x + zone_width)
            start_y = max(0, height - zone_height)
            end_y = height
            
            if self.debug_mode:
                self.get_logger().info(f"Safety zone: [{start_y}:{end_y}, {start_x}:{end_x}]")
            
            # Extract safety zone
            safety_zone = depth_image[start_y:end_y, start_x:end_x]
            
            # Remove invalid depth values (NaN, inf, 0)
            valid_mask = np.isfinite(safety_zone) & (safety_zone > 0.1) & (safety_zone < 10.0)  # Reasonable depth range
            valid_depths = safety_zone[valid_mask]
            
            if len(valid_depths) > 0:
                # Find minimum distance in safety zone
                self.min_distance = np.min(valid_depths)
                mean_distance = np.mean(valid_depths)
                
                # Check if obstacle is detected
                prev_obstacle = self.obstacle_detected
                self.obstacle_detected = self.min_distance < self.obstacle_threshold
                
                if self.debug_mode:
                    self.get_logger().info(f"Valid depths: {len(valid_depths)}, Min: {self.min_distance:.2f}m, Mean: {mean_distance:.2f}m")
                
                current_time = self.get_clock().now()
                
                if self.obstacle_detected:
                    if not prev_obstacle or (current_time - self.last_log_time).nanoseconds > 1e9:
                        self.get_logger().warn(f"OBSTACLE DETECTED! Distance: {self.min_distance:.2f}m (threshold: {self.obstacle_threshold}m)")
                        self.last_log_time = current_time
                    self.last_obstacle_time = current_time
                else:
                    if prev_obstacle or (current_time - self.last_log_time).nanoseconds > 3e9:
                        self.get_logger().info(f"Path clear - closest: {self.min_distance:.2f}m")
                        self.last_log_time = current_time
            else:
                # No valid depth data
                self.obstacle_detected = False
                self.min_distance = float('inf')
                current_time = self.get_clock().now()
                if (current_time - self.last_log_time).nanoseconds > 3e9:
                    self.get_logger().warn("No valid depth data in safety zone")
                    self.last_log_time = current_time
                
        except Exception as e:
            self.get_logger().error(f"Error processing depth image: {e}")
    
    def update_speed_factor(self):
        """Gradually adjust speed factor based on obstacle detection"""
        prev_factor = self.current_speed_factor
        
        if self.obstacle_detected:
            # Gradually decrease speed when obstacle detected
            target_factor = self.min_speed_factor
            if self.current_speed_factor > target_factor:
                self.current_speed_factor = max(
                    target_factor,
                    self.current_speed_factor - self.max_deceleration_rate
                )
        else:
            # Gradually increase speed when path is clear
            target_factor = 1.0
            if self.current_speed_factor < target_factor:
                self.current_speed_factor = min(
                    target_factor,
                    self.current_speed_factor + self.max_acceleration_rate
                )
        
        # Ensure speed factor stays within bounds
        self.current_speed_factor = max(0.0, min(1.0, self.current_speed_factor))
        
        if self.debug_mode and abs(self.current_speed_factor - prev_factor) > 0.01:
            self.get_logger().info(f"Speed factor changed: {prev_factor:.2f} -> {self.current_speed_factor:.2f}")
    
    def publish_modified_velocity(self):
        """Publish modified velocity commands with obstacle avoidance"""
        # Update speed factor based on current obstacle status
        self.update_speed_factor()
        
        # Create modified velocity command
        self.modified_cmd_vel.linear.x = self.current_cmd_vel.linear.x * self.current_speed_factor
        self.modified_cmd_vel.linear.y = self.current_cmd_vel.linear.y * self.current_speed_factor
        self.modified_cmd_vel.linear.z = self.current_cmd_vel.linear.z * self.current_speed_factor
        
        # Reduce angular velocity when obstacle detected but keep some maneuverability
        if self.obstacle_detected:
            angular_factor = max(0.5, self.current_speed_factor)  # Keep more turning ability
        else:
            angular_factor = self.current_speed_factor
            
        self.modified_cmd_vel.angular.x = self.current_cmd_vel.angular.x * angular_factor
        self.modified_cmd_vel.angular.y = self.current_cmd_vel.angular.y * angular_factor
        self.modified_cmd_vel.angular.z = self.current_cmd_vel.angular.z * angular_factor
        
        # Always publish, even if zero
        self.cmd_vel_pub.publish(self.modified_cmd_vel)
        
        # Debug output for significant changes
        if self.debug_mode and (abs(self.current_cmd_vel.linear.x) > 0.01 or abs(self.modified_cmd_vel.linear.x) > 0.01):
            self.get_logger().info(f"Publishing: input={self.current_cmd_vel.linear.x:.2f} -> output={self.modified_cmd_vel.linear.x:.2f} (factor={self.current_speed_factor:.2f})")

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = ObstacleAvoidanceNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()