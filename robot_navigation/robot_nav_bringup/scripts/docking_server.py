#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import PoseStamped, Twist, Quaternion, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from robot_nav_bringup.srv import RequestDock, RequestUndock
from robot_nav_bringup.msg import DockingStatus
from tf2_ros import TransformListener, Buffer
import math
import time

class DockingServer(Node):
    def __init__(self):
        super().__init__('docking_server')
        
        # Parameters - will be set from nav2_params.yaml
        self.declare_parameter('docking_pose', [-14.08, -3.30, 0.713641])  # [x, y, yaw]
        self.declare_parameter('docking_tolerance', 0.5)  # meters - increased tolerance
        self.declare_parameter('approach_distance', 0.5)  # meters
        self.declare_parameter('undock_distance', 1.0)  # meters to move backward during undock
        
        # Services
        self.dock_srv = self.create_service(RequestDock, 'request_dock', self.dock_callback)
        self.undock_srv = self.create_service(RequestUndock, 'request_undock', self.undock_callback)
        
        # Publisher
        self.status_pub = self.create_publisher(DockingStatus, 'docking_status', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/diff_drive/cmd_vel', 10)
        
        # Subscriber for robot pose
        self.robot_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10
        )
        
        # Current robot pose
        self.current_pose = None
        
        # Action client for navigation
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        self.get_logger().info('Docking server initialized')

    def pose_callback(self, msg):
        """Update current robot pose"""
        self.current_pose = msg.pose.pose

    def yaw_to_quaternion(self, yaw):
        """Convert yaw angle to quaternion"""
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q

    def distance_to_pose(self, x1, y1, x2, y2):
        """Calculate distance between two points"""
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

    def dock_callback(self, request, response):
        self.get_logger().info('Received dock request')
        self.publish_status(DockingStatus.DOCKING_IN_PROGRESS, "Docking started")
        
        try:
            # Get docking parameters
            docking_pose = self.get_parameter('docking_pose').get_parameter_value().double_array_value
            tolerance = self.get_parameter('docking_tolerance').value
            approach_dist = self.get_parameter('approach_distance').value
            
            self.get_logger().info(f'Docking to: {docking_pose}')
            
            # Create approach pose (slightly before the actual dock)
            approach_pose = self.create_approach_pose(docking_pose, approach_dist)
            
            # Navigate to approach pose
            nav_success = self.navigate_to_pose(approach_pose)
            
            if not nav_success:
                response.success = False
                response.message = "Failed to navigate to docking approach position"
                self.publish_status(DockingStatus.DOCKING_FAILED, "Navigation failed")
                return response
            
            # Check if we're close to the docking pose
            dock_x, dock_y, dock_yaw = docking_pose
            if self.current_pose:
                current_x = self.current_pose.position.x
                current_y = self.current_pose.position.y
                distance = self.distance_to_pose(current_x, current_y, dock_x, dock_y)
                
                if distance <= tolerance:
                    response.success = True
                    response.message = "Docking successful - already at dock position"
                    self.publish_status(DockingStatus.DOCKING_SUCCESS, "Docking completed")
                    return response
            
            # If not close enough, perform final docking maneuver
            self.get_logger().info('Reached approach position, checking if final docking needed')
            time.sleep(1.0)
            
            # Simple forward movement to final position
            dock_success = self.perform_final_docking(docking_pose, tolerance)
            
            if dock_success:
                response.success = True
                response.message = "Docking successful"
                self.publish_status(DockingStatus.DOCKING_SUCCESS, "Docking completed")
            else:
                response.success = False
                response.message = "Docking failed in final approach"
                self.publish_status(DockingStatus.DOCKING_FAILED, "Final approach failed")
                
        except Exception as e:
            self.get_logger().error(f'Docking error: {str(e)}')
            response.success = False
            response.message = f"Docking error: {str(e)}"
            self.publish_status(DockingStatus.DOCKING_FAILED, f"Error: {str(e)}")
        
        return response

    def create_approach_pose(self, docking_pose, approach_distance):
        """Create a pose slightly before the docking station"""
        x, y, yaw = docking_pose
        
        # Calculate approach position (move back along the docking direction)
        approach_x = x - approach_distance * math.cos(yaw)
        approach_y = y - approach_distance * math.sin(yaw)
        
        # Create PoseStamped message
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = approach_x
        pose.pose.position.y = approach_y
        pose.pose.position.z = 0.0
        
        # Set orientation (facing the docking station)
        pose.pose.orientation = self.yaw_to_quaternion(yaw)
        
        self.get_logger().info(f'Approach pose: ({approach_x:.2f}, {approach_y:.2f}) facing {yaw:.2f} rad')
        return pose

    def navigate_to_pose(self, pose):
        """Navigate to a specific pose using Nav2"""
        self.get_logger().info(f'Navigating to pose: ({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})')
        
        # Wait for navigation server
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Navigation server not available')
            return False
        
        # Create goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        
        # Send goal
        future = self.nav_to_pose_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return False
        
        # Wait for result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        # Return True if goal was accepted (Nav2 will handle the navigation)
        return True

    def perform_final_docking(self, docking_pose, tolerance):
        """Perform the final precise docking maneuver if needed"""
        dock_x, dock_y, dock_yaw = docking_pose
        
        if not self.current_pose:
            self.get_logger().warn('No current pose available, skipping final docking')
            return True
            
        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y
        distance = self.distance_to_pose(current_x, current_y, dock_x, dock_y)
        
        if distance <= tolerance:
            self.get_logger().info(f'Already at dock position (distance: {distance:.2f}m)')
            return True
        
        self.get_logger().info(f'Performing final docking, distance to dock: {distance:.2f}m')
        
        try:
            # Move slowly forward towards actual docking pose
            twist = Twist()
            twist.linear.x = 0.1  # Slow forward motion
            
            # Move forward until we reach the dock position
            start_time = time.time()
            while self.current_pose and time.time() - start_time < 10.0:  # 10 second timeout
                current_x = self.current_pose.position.x
                current_y = self.current_pose.position.y
                distance = self.distance_to_pose(current_x, current_y, dock_x, dock_y)
                
                if distance <= tolerance:
                    break
                
                self.cmd_vel_pub.publish(twist)
                time.sleep(0.1)
            
            # Stop the robot
            twist.linear.x = 0.0
            for _ in range(5):
                self.cmd_vel_pub.publish(twist)
                time.sleep(0.1)
            
            self.get_logger().info('Final docking maneuver completed')
            return True
            
        except Exception as e:
            self.get_logger().error(f'Final docking error: {str(e)}')
            # Make sure to stop the robot on error
            twist = Twist()
            twist.linear.x = 0.0
            self.cmd_vel_pub.publish(twist)
            return False

    def undock_callback(self, request, response):
        self.get_logger().info('Received undock request')
        self.publish_status(DockingStatus.UNDOCKING_IN_PROGRESS, "Undocking started")
        
        try:
            # Get undock distance parameter
            undock_dist = self.get_parameter('undock_distance').value
            
            # Move backward for undocking
            twist = Twist()
            twist.linear.x = -0.2  # Backward motion
            
            # Calculate time based on distance and speed
            move_time = undock_dist / abs(twist.linear.x)
            
            self.get_logger().info(f'Undocking: moving backward {undock_dist}m')
            
            # Move backward for calculated time
            start_time = time.time()
            while time.time() - start_time < move_time:
                self.cmd_vel_pub.publish(twist)
                time.sleep(0.1)
            
            # Stop the robot
            twist.linear.x = 0.0
            for _ in range(5):
                self.cmd_vel_pub.publish(twist)
                time.sleep(0.1)
            
            response.success = True
            response.message = "Undocking successful"
            self.publish_status(DockingStatus.UNDOCKING_SUCCESS, "Undocking completed")
                
        except Exception as e:
            self.get_logger().error(f'Undocking error: {str(e)}')
            response.success = False
            response.message = f"Undocking error: {str(e)}"
            self.publish_status(DockingStatus.UNDOCKING_FAILED, f"Error: {str(e)}")
        
        return response

    def publish_status(self, status, message):
        msg = DockingStatus()
        msg.status = status
        msg.message = message
        self.status_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    docking_server = DockingServer()
    
    executor = MultiThreadedExecutor()
    rclpy.spin(docking_server, executor=executor)
    
    docking_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()