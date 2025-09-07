# Copyright 2022 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
def generate_launch_description():
    # Configure ROS nodes for launch

    # Setup project paths
    pkg_project_bringup = get_package_share_directory('robot_nav_bringup')
    pkg_project_gazebo = get_package_share_directory('robot_nav_gazebo')
    pkg_project_description = get_package_share_directory('robot_nav_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    urdf_file = os.path.join(pkg_project_description, 'models', 'diff_drive', 'model.urdf')
    with open(urdf_file, 'r') as infp:
        rob_desc = infp.read()  # This should be URDF content

    # Setup to launch the simulator and Gazebo world    
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': PathJoinSubstitution([
            pkg_project_gazebo,
            'worlds',
            'diff_drive.sdf'
        ])}.items(),
    )

    # Takes the description and joint angles as inputs and publishes the 3D poses of the robot links
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': rob_desc},
            {'frame_prefix': 'diff_drive/'}
        ]
    )

    # REMOVE ros2_control_node - not needed for Gazebo simulation
    # REMOVE controller spawners - not needed

    # Visualize in RViz
    rviz = Node(
       package='rviz2',
       executable='rviz2',
       arguments=['-d', os.path.join(pkg_project_bringup, 'config', 'diff_drive.rviz')],
       condition=IfCondition(LaunchConfiguration('rviz'))
    )

    # Bridge ROS topics and Gazebo messages for establishing communication
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_project_bringup, 'config', 'robot_nav_bridge.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen'
    )
    static_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'diff_drive', 'diff_drive/chassis'],
        output='screen'
    )
    # SLAM Toolbox with corrected frame parameters
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'scan_topic': '/diff_drive/scan',
            'base_frame': 'diff_drive',
            'odom_frame': 'diff_drive/odom',
            'map_frame': 'map',
            
            # Fix timing issues
            'transform_timeout': 2.0,
            'tf_buffer_duration': 5.0,
            'minimum_time_interval': 0.1,  # Changed from 0.5 to match 10Hz lidar
            'transform_publish_period': 0.02,
            
            # Lidar parameters to match your sensor
            'resolution': 0.05,
            'max_laser_range': 10.0,
            'minimum_travel_distance': 0.2,  # Add this
            'minimum_travel_heading': 0.2,   # Add this
            'minimum_laser_range': 0.1,  # Add this - critical!

            # SLAM algorithm parameters
            'loop_match_minimum_chain_size': 10,
            'loop_match_maximum_variance_coarse': 3.0,
            'correlation_search_space_dimension': 0.5,
            'correlation_search_space_resolution': 0.01,
            
            # Processing parameters
            'throttle_scans': 1,  # Process every scan
            'map_update_interval': 5.0,
        }]
    )


    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Open RViz.'),
        DeclareLaunchArgument('debug', default_value='false',
                              description='Enable debug tools.'),
        
        # Core nodes
        gz_sim,
        bridge,
        robot_state_publisher,
        
        # SLAM
        slam_toolbox_node,
        # static_transform,
        # Visualization
        rviz,
    ])