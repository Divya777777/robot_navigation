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

    # Visualize in RViz
    rviz = Node(
       package='rviz2',
       executable='rviz2',
       arguments=['-d', os.path.join(pkg_project_bringup, 'config', 'basic_nav.rviz')],
       condition=IfCondition(LaunchConfiguration('rviz'))
    )

    # Bridge ROS topics and Gazebo messages for establishing communication
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
    # Add this to your second launch file, before the return statement
    cmd_vel_relay = Node(
        package='topic_tools',
        executable='relay',
        name='cmd_vel_relay',
        parameters=[{
            'input_topic': 'cmd_vel_nav',
            'output_topic': '/diff_drive/cmd_vel',
            'monitor_rate': 20.0
        }],
        output='screen'
    )
    # Static transform publishers
    odom_to_base_footprint_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'diff_drive/odom', 'diff_drive/base_footprint'],
        output='screen'
    )
    # Add this to your launch file as a temporary test
    map_to_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'diff_drive/odom'],
        output='screen'
    )
    base_footprint_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'diff_drive/base_footprint', 'diff_drive/chassis'],
        output='screen'
    )

    # SOLUTION 1: Single combined navigation launch (RECOMMENDED)
    # This includes both localization and navigation in one launch file
    combined_nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch'),
            '/bringup_launch.py'
        ]),
        launch_arguments={
            'map': os.path.join(pkg_project_bringup, 'maps', 'room1.yaml'),
            'use_sim_time': 'True',
            'params_file': os.path.join(pkg_project_bringup, 'config', 'nav2_params.yaml')
        }.items(),
        condition=IfCondition(LaunchConfiguration('navigation'))
    )

    # SOLUTION 2: Sequential launch with event handler (ALTERNATIVE)
    # Uncomment this section and comment out SOLUTION 1 if you prefer separate launches
    """
    # Localization launch 
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_project_bringup, 'launch', 'localization.launch.py')),
        launch_arguments={
            'map': os.path.join(pkg_project_bringup, 'maps', 'my_map.yaml'),
            'use_sim_time': 'True',
            'params_file': os.path.join(pkg_project_bringup, 'config', 'nav2_params.yaml')
        }.items(),
        condition=IfCondition(LaunchConfiguration('localization'))
    )

    # Navigation launch that starts after localization
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_project_bringup, 'launch', 'navigation.launch.py')),
        launch_arguments={
            'use_sim_time': 'True',
            'params_file': os.path.join(pkg_project_bringup, 'config', 'nav2_params.yaml'),
            'use_lifecycle_mgr': 'false'  # Don't start another lifecycle manager
        }.items(),
        condition=IfCondition(LaunchConfiguration('navigation'))
    )

    # Event handler to start navigation after localization
    navigation_event_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=localization_launch,
            on_exit=[navigation_launch],
        ),
        condition=IfCondition(LaunchConfiguration('navigation'))
    )
    """
    # lidar_tf = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     arguments=['0', '0', '0.5', '0', '0', '0', 'diff_drive/chassis', 'diff_drive/lidar_link'],
    #     output='screen'
    # )
    pointcloud_filter = Node(
        package='robot_nav_bringup',
        executable='pointcloud_filter.py',
        name='pointcloud_filter',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Open RViz.'),
        DeclareLaunchArgument('debug', default_value='false',
                              description='Enable debug tools.'),
        DeclareLaunchArgument('localization', default_value='false',
                              description='Launch only localization (deprecated - use navigation instead).'),
        DeclareLaunchArgument('navigation', default_value='true',
                              description='Launch full navigation stack (includes localization).'),
        
        # Core nodes
        base_footprint_tf,
        cmd_vel_relay,
        map_to_odom_tf,
        odom_to_base_footprint_tf,
        
        
        gz_sim,
        bridge,
        robot_state_publisher,
        rviz,
        
        # Navigation (includes localization)
        combined_nav_launch,
        pointcloud_filter
        # Uncomment these lines if using SOLUTION 2 instead
        # localization_launch,
        # navigation_event_handler,
    ])