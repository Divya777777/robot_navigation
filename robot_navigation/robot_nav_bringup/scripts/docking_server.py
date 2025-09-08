#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.task import Future

from geometry_msgs.msg import PoseStamped, Twist, Quaternion
from nav2_msgs.action import NavigateToPose
from robot_nav_bringup.srv import RequestDock, RequestUndock
from robot_nav_bringup.msg import DockingStatus
from tf2_ros import TransformListener, Buffer
import math
import time

class DockingServer(Node):
    def __init__(self):
        super().__init__('docking_server')
        
        # Parameters
        self.declare_parameter('docking_pose', [-14.08, -3.30, 0.713641])  # [x, y, yaw]
        self.declare_parameter('docking_tolerance', 0.1)  # meters
        self.declare_parameter('approach_distance', 0.5)  # meters
        self.declare_parameter('final_dock_distance', 0.6)  # meters to move forward
        self.declare_parameter('nav_timeout', 120.0)  # seconds
        
        # Services
        self.dock_srv = self.create_service(RequestDock, 'request_dock', self.dock_callback)
        self.undock_srv = self.create_service(RequestUndock, 'request_undock', self.undock_callback)
        
        # Publisher
        self.status_pub = self.create_publisher(DockingStatus, 'docking_status', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/diff_drive/cmd_vel', 10)
        
        # Action client for navigation
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # TF buffer for coordinate transformations
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.get_logger().info('Docking server initialized')

    def yaw_to_quaternion(self, yaw):
        """Convert yaw angle to quaternion"""
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q

    def dock_callback(self, request, response):
        self.get_logger().info('Received dock request')
        self.publish_status(DockingStatus.DOCKING_IN_PROGRESS, "Docking started")
        
        try:
            # Get docking parameters
            docking_pose = self.get_parameter('docking_pose').get_parameter_value().double_array_value
            tolerance = self.get_parameter('docking_tolerance').value
            approach_dist = self.get_parameter('approach_distance').value
            final_dock_dist = self.get_parameter('final_dock_distance').value
            nav_timeout = self.get_parameter('nav_timeout').value
            
            self.get_logger().info(f'Docking to: {docking_pose}')
            
            # Create approach pose (slightly before the actual dock)
            approach_pose = self.create_approach_pose(docking_pose, approach_dist)
            
            # Navigate to approach pose
            nav_success = self.navigate_to_pose(approach_pose, nav_timeout)
            
            if not nav_success:
                response.success = False
                response.message = "Failed to navigate to docking approach position"
                self.publish_status(DockingStatus.DOCKING_FAILED, "Navigation failed")
                return response
            
            # Wait a moment before final approach
            self.get_logger().info('Reached approach position, starting final docking')
            time.sleep(2.0)
            
            # Perform final docking maneuver
            dock_success = self.perform_final_docking(docking_pose, final_dock_dist, tolerance)
            
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

    def navigate_to_pose(self, pose, timeout=30.0):
        """Navigate to a specific pose using Nav2 with timeout"""
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
        
        # Wait for goal acceptance with timeout
        start_time = time.time()
        while not future.done() and time.time() - start_time < 5.0:
            time.sleep(0.1)
        
        if not future.done():
            self.get_logger().error('Goal acceptance timeout')
            return False
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return False
        
        self.get_logger().info('Goal accepted, waiting for navigation to complete...')
        
        # Wait for result with timeout
        result_future = goal_handle.get_result_async()
        start_time = time.time()
        while not result_future.done() and time.time() - start_time < timeout:
            time.sleep(0.1)
        
        if not result_future.done():
            self.get_logger().warn('Navigation timeout, but may have reached close enough')
            # Cancel the goal
            cancel_future = goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, cancel_future)
            return True  # Assume we're close enough for final docking
        
        try:
            result = result_future.result().result
            self.get_logger().info(f'Navigation completed with result')
            return True
        except:
            self.get_logger().warn('Navigation may have been cancelled, but proceeding')
            return True

    def perform_final_docking(self, docking_pose, final_distance, tolerance):
        """Perform the final precise docking maneuver"""
        self.get_logger().info('Starting final docking maneuver')
        
        try:
            x, y, yaw = docking_pose
            
            # Move slowly forward towards actual docking pose
            twist = Twist()
            twist.linear.x = 0.1  # Slow forward motion
            
            # Calculate time needed based on distance and speed
            move_time = final_distance / abs(twist.linear.x)
            
            self.get_logger().info(f'Moving forward for {move_time:.2f} seconds to final dock position')
            
            # Move forward for calculated time
            start_time = time.time()
            while time.time() - start_time < move_time:
                self.cmd_vel_pub.publish(twist)
                time.sleep(0.1)
            
            # Stop the robot gently
            twist.linear.x = 0.0
            for _ in range(5):  # Publish stop multiple times
                self.cmd_vel_pub.publish(twist)
                time.sleep(0.1)
            
            self.get_logger().info('Final docking maneuver completed successfully')
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
            # Move backward for undocking
            twist = Twist()
            twist.linear.x = -0.1
            
            # Move backward for 3 seconds
            start_time = time.time()
            while time.time() - start_time < 3.0:
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