#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from sensor_msgs.msg import PointField
import numpy as np
import struct

class PointCloudFilter(Node):
    def __init__(self):
        super().__init__('pointcloud_filter')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/diff_drive/rgbd/points',
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(PointCloud2, '/filtered_points', 10)
        self.voxel_size = 0.08  # Increased for better performance
        self.max_range = 5.0    # Maximum range to process
        self.min_range = 0.1    # Minimum range to process
        self.get_logger().info('PointCloud Filter node started')

    def listener_callback(self, msg):
        try:
            # Read points with additional fields to detect invalid points
            points = list(pc2.read_points(
                msg, 
                field_names=("x", "y", "z"), 
                skip_nans=False  # Don't skip, we'll handle manually
            ))
            
            if not points:
                return
            
            # Filter out invalid points (NaN, inf, out of range)
            valid_points = []
            for point in points:
                x, y, z = point
                
                # Check for NaN or infinite values
                if (np.isnan(x) or np.isnan(y) or np.isnan(z) or
                    np.isinf(x) or np.isinf(y) or np.isinf(z)):
                    continue
                
                # Check range limits
                distance = np.sqrt(x**2 + y**2 + z**2)
                if distance < self.min_range or distance > self.max_range:
                    continue
                
                valid_points.append((x, y, z))
            
            if not valid_points:
                self.get_logger().debug('No valid points after filtering')
                return
            
            # Voxel filtering
            voxel_grid = {}
            for point in valid_points:
                x, y, z = point
                
                # Create voxel key
                voxel_key = (
                    round(x / self.voxel_size) * self.voxel_size,
                    round(y / self.voxel_size) * self.voxel_size,
                    round(z / self.voxel_size) * self.voxel_size
                )
                
                # Keep the first point in each voxel
                if voxel_key not in voxel_grid:
                    voxel_grid[voxel_key] = point
            
            # Create filtered point cloud
            filtered_points = list(voxel_grid.values())
            
            self.get_logger().debug(
                f'Filtered: {len(points)} -> {len(valid_points)} -> {len(filtered_points)} points'
            )
            
            # Create new PointCloud2 message
            fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            ]
            
            header = msg.header
            filtered_msg = pc2.create_cloud(header, fields, filtered_points)
            self.publisher.publish(filtered_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error processing point cloud: {str(e)}', throttle_duration_sec=5)

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()