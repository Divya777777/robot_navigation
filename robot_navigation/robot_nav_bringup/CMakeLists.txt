cmake_minimum_required(VERSION 3.8)
project(robot_nav_bringup)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# Find message generation packages
find_package(rosidl_default_generators REQUIRED)

# Project specific dependencies
find_package(robot_nav_description REQUIRED)
find_package(robot_nav_gazebo REQUIRED)

# Generate messages and services
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DockingStatus.msg"
  "srv/RequestDock.srv"
  "srv/RequestUndock.srv"
  DEPENDENCIES std_msgs
)

# Add include directories
include_directories(
  include
  ${ament_INCLUDE_DIRS}
)

# Install Python scripts
install(PROGRAMS
  scripts/pointcloud_filter.py
  scripts/obstacle_safety_node.py
  scripts/docking_server.py
  scripts/test_docking_client.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(
  DIRECTORY
    launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install configuration files
install(
  DIRECTORY
    config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install map files
install(
  DIRECTORY
    maps/
  DESTINATION share/${PROJECT_NAME}/maps
)

# Install message and service files
install(
  DIRECTORY
    msg/
  DESTINATION share/${PROJECT_NAME}/msg
)

install(
  DIRECTORY
    srv/
  DESTINATION share/${PROJECT_NAME}/srv
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()