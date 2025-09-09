import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
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
       condition=IfCondition(LaunchConfiguration('rviz')),
       parameters=[{'use_sim_time': True}]  # ✅ ADDED: use_sim_time for RViz
    )
    
    docking_server = Node(
        package='robot_nav_bringup',
        executable='docking_server.py',
        name='docking_server',
        output='screen',
        parameters=[{'use_sim_time': True},
                    os.path.join(pkg_project_bringup, 'config', 'nav2_params.yaml')]
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
    
    # ✅ COMMENT OUT: This static transform conflicts with AMCL
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

    # ✅ ADDED: Lidar transform (CRITICAL for laser scan processing)
    lidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0.2', '0', '0', '0', 'diff_drive/chassis', 'diff_drive/lidar_link'],
        output='screen'
    )

    ekf_config_path = PathJoinSubstitution([pkg_project_bringup, 'config', 'ekf.yaml'])
    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path, {'use_sim_time': True,'scan_topic': '/diff_drive/scan',
                                    'base_frame': 'diff_drive/chassis',
                                    'odom_frame': 'diff_drive/odom'}],
        remappings=[
            ('/odometry/filtered', '/diff_drive/odometry/filtered'),
            ('/odom/diff_drive', '/diff_drive/odom')  # ✅ Ensure proper odom topic
        ]
    )
    
    # Slip detector node
    slip_detector_node = Node(
        package='robot_nav_bringup',
        executable='slip_detector.py',
        name='slip_detector',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # ✅ ADDED: Delayed navigation launch to ensure TF is established
    delayed_navigation = TimerAction(
        period=10.0,  # Wait 10 seconds for TF to stabilize
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(get_package_share_directory('nav2_bringup'), 'launch'),
                    '/bringup_launch.py'
                ]),
                launch_arguments={
                    'map': os.path.join(pkg_project_bringup, 'maps', 'room2.yaml'),
                    'use_sim_time': 'True',
                    'params_file': os.path.join(pkg_project_bringup, 'config', 'nav2_params.yaml'),
                    'autostart': 'true'  # ✅ Ensure autostart is enabled
                }.items(),
                condition=IfCondition(LaunchConfiguration('navigation'))
            )
        ]
    )

    pointcloud_filter = Node(
        package='robot_nav_bringup',
        executable='pointcloud_filter.py',
        name='pointcloud_filter',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # ✅ ADDED: Event handler to ensure proper startup sequence
    navigation_event_handler = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=bridge,
            on_start=[delayed_navigation]
        )
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
        gz_sim,
        bridge,
        robot_state_publisher,
        
        # TF transforms (CRITICAL for timing)
        map_to_odom_tf,  # ✅ COMMENTED OUT: Conflicts with AMCL
        base_footprint_tf,
        odom_to_base_footprint_tf,
        lidar_tf,  # ✅ ADDED: Lidar transform
        
        # Localization nodes
        robot_localization_node,
        slip_detector_node,
        
        # Other functionality
        cmd_vel_relay,
        docking_server,
        pointcloud_filter,
        rviz,
        
        # ✅ Navigation with proper timing
        navigation_event_handler,
    ])