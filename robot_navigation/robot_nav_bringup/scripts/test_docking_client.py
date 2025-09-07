#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from robot_nav_bringup.srv import RequestDock, RequestUndock

class DockingClient(Node):
    def __init__(self):
        super().__init__('docking_client')
        self.dock_client = self.create_client(RequestDock, 'request_dock')
        self.undock_client = self.create_client(RequestUndock, 'request_undock')
        
        while not self.dock_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Dock service not available, waiting...')
        
        self.get_logger().info('Docking client ready')

    def call_dock(self):
        request = RequestDock.Request()
        future = self.dock_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def call_undock(self):
        request = RequestUndock.Request()
        future = self.undock_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

def main(args=None):
    rclpy.init(args=args)
    client = DockingClient()
    
    # Test docking
    result = client.call_dock()
    client.get_logger().info(f'Dock result: {result.success} - {result.message}')
    
    # Test undocking
    result = client.call_undock()
    client.get_logger().info(f'Undock result: {result.success} - {result.message}')
    
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()