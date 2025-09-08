# Robot Navigation (ROS 2 Humble + Gazebo Harmonic)

> **Goal:** Simulate a differential-drive robot in Gazebo **Harmonic**, generate a 2D occupancy **map** with **SLAM Toolbox** by teleoperating the robot, Run full **navigation** using the generated map, Do the obstacle avoidance using depth camera and sensor fusion and create a service to do the docking of robot

---


## 📦 Prerequisites

* **ROS 2 Humble** installed
* **Gazebo Harmonic** (Ignition/GZ) installed

  * `gz sim`, `ros_gz_sim`, `ros_gz_bridge`
* **Nav2** (Navigation2) stack: `nav2_bringup`, `nav2_map_server`, etc.
* **SLAM Toolbox**: `slam_toolbox`
* **Teleop**: `teleop_twist_keyboard`
* **colcon** build tools
* **rviz2** for visualization

---

### 1. Building the Package

1.  **Create a ROS 2 workspace (if needed):**
    ```bash
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws/src
    ```
2.  **Clone the repository into your `src` folder:**
    ```bash
    git clone https://github.com/Divya777777/robot_navigation
    ```
3.  **Build the package:**
    ```bash
    cd ~/ros2_ws
    colcon build
    ```
4.  **Source your workspace:**
    ```bash
    source install/setup.bash
    ```

## 🗺️ Map Generation Workflow
![](https://github.com/Divya777777/2d_map_to_gazebo_world/blob/humble/Gifs/2d_to_world.gif)

Generate the map with:

```bash
ros2 launch robot_nav_bringup map_genration.launch.py
```

This launch file:

1. Starts **Gazebo Harmonic** with the `house.world.sdf` world
2. Spawns the **model.urdf**
3. Starts **SLAM Toolbox** in online mode
4. Opens **RViz2** (if configured)

### Teleopkeyboard to drive the Robot

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=false --remap /cmd_vel:=/diff_drive/cmd_vel
```

### Save the Map

```bash
ros2 run nav2_map_server map_saver_cli -f your/robot_nav_bringup/maps/map_name
```

---

## 🚀 Navigation Workflow
![](https://github.com/Divya777777/2d_map_to_gazebo_world/blob/humble/Gifs/2d_to_world.gif)

Once a map has been generated and saved, start navigation with:

```bash
ros2 launch robot_nav_bringup rrbot_setup.launch.py
```

This launch file:

1. Starts **Gazebo Harmonic** with the `house.world.sdf` world
2. Spawns the **diff‑drive robot** with sensors
3. Launches **Nav2 bringup** configured for localization and navigation (`localization.launch.py` and   `navigation.launch.py `)
4. Loads the saved map (`maps/house.yaml`)
5. Loads parameters from `nav2_params.yaml`
6. Opens **RViz2** for interaction
7. Docking server for dock services
   
Use RViz2’s **2D Goal Pose** tool to send navigation goals.

---

## 🛑 Obstacle Avoidance (Depth Camera + Lidar Fusion)
![](https://github.com/Divya777777/2d_map_to_gazebo_world/blob/humble/Gifs/2d_to_world.gif)

This project uses **sensor fusion** for obstacle detection and avoidance:

* **Lidar (`/diff_drive/scan`)**  
  Provides 2D LaserScan data for detecting obstacles in the robot’s horizontal plane.

* **Depth Camera (`/filtered_points`)**  
  Publishes a `PointCloud2` stream that captures 3D depth data in front of the robot.  
  Integrated into both local and global costmaps through the **VoxelLayer** plugin.

### How it Works

1. **Local Costmap**
   * Uses `voxel_layer` and `inflation_layer`.
   * `observation_sources`: `scan`, `depth_camera`
   * Depth camera (`PointCloud2`) allows detection of obstacles not visible to Lidar (e.g., low/high objects).
   * `robot_radius` and inflation radius expanded to add conservative safety margins.

2. **Global Costmap**
   * Uses `static_layer` (for map), `obstacle_layer`, and `inflation_layer`.
   * Merges both Lidar and depth camera data for path planning across the whole map.

3. **Voxel Layer**
   * 3D representation of the environment is maintained using `z_resolution`, `z_voxels`.
   * `marking` and `clearing` ensure dynamic obstacles are updated live.

---

## 📍 Config Files Explained


### `nav2_params.yaml`

* Parameters for Nav2 navigation stack:

  * **amcl**: localization on saved map
  * **planner\_server**: global planner config
  * **controller\_server**: local planner config
  * **bt\_navigator**: behavior tree for navigation
  * **recoveries\_server**: recovery behaviors (rotate, clear costmap)

### `basic_nav.rviz`

* Preconfigured RViz2 layout for navigation launch file:

  * Displays for Map, LaserScan, TF, RobotModel, etc
  * Helpful for monitoring SLAM and navigation

### `diff_drive.rviz`

* Preconfigured RViz2 layout for map genration launch file:

  * Displays for Map, LaserScan, RobotModel, etc
  * Helpful for monitoring SLAM and navigation

---

## ⚓ Service-Based Docking  
![](https://github.com/Divya777777/2d_map_to_gazebo_world/blob/humble/Gifs/2d_to_world.gif)

This project also includes a **custom ROS 2 service** to trigger **auto-docking** of the robot.  
Instead of publishing goals manually in RViz, a service call can be used to send the robot to a predefined **docking station pose**.  

### How It Works
1. A service (e.g., `DockRobot`) is created using `rclpy`.  
2. When the service is called, the node publishes a `NavigateToPose` goal to the Nav2 stack.  
3. The goal corresponds to the docking station’s saved coordinates (`x`, `y`, `yaw`) on the map.  
4. Nav2 plans and executes the path automatically.  

### Example Service Call
```bash
ros2 service call /request_dock robot_nav_bringup/srv/RequestDock
```
We can run this docking server externally using 
```bash
ros2 run robot_nav_bringup docking_server.py
```
This file is alredy included in `navigation.launch.py` so we don't have to run it externally