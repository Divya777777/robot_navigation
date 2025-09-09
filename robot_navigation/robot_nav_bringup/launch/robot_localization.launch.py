
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Get the launch directory
    pkg_dir = get_package_share_directory('robot_nav_bringup')  # Replace with your package name
    
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use sim time if true'
    )
    
    # Robot localization EKF node
    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            os.path.join(pkg_dir, 'config', 'ekf.yaml'),
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('/odometry/filtered', '/diff_drive/odometry/filtered')
        ]
    )
    
    # Create the launch description and populate
    ld = LaunchDescription()
    
    # Add the actions to the launch description
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(robot_localization_node)
    
    return ld
