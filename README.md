## Autonomous Navigation with Obstacle Avoidance

This package provides a **simple differential drive robot** that can autonoumously navigate its surroundings and perform obstacle detection and avoidance using **Nav2 on a pre-generated map**. You can also generate a map using the bot by driving it around the surroundings (using slam_toolbox).

### Software Stack and Packages Used

- **ROS2 Jazzy on Ubuntu 24.04.1 LTS**
- **Mapping using LiDAR-SLAM** : `slam_toolbox`
- **Autonoumous Navigation and Obstacle Avoidance** : `Nav2`
- **Localization** : `nav2_amcl`
- **Sensor Fusion**: `robot_localization`
- **Simulation** : `Gazebo Sim`
- **URDF and State Publishing**: `joint_state_publisher` and `robot_state_publisher`

---

### Setup

```bash
cd ~
mkdir -p ros2_ws/src
cd ros2_ws/src
git clone <repository_url>
cd ..
colcon build
. install/setup.bash
```

The package is now ready for use

---

### Mapping the Area Using the Bot

1. **Launch the robot model :**

```
ros2 launch diff_car_model map.launch.py
```

2. **Launch SLAM Toolbox** in a separate terminal:

```bash
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True slam_params_file:=$PWD/src/diff_car_model/config/slam_toolbox.yaml
```

3. **Control the robot manually**: In another terminal, use the `teleop_twist_keyboard` package to drive the robot in the Gazebo simulation:

```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Follow the on-screen instructions to operate the bot using your keyboard and drive it around to generate a map.

4.  **Save the map :**

- Open the SLAM Toolbox panel in RViz.
- Click the "Save Map" button to save the generated map.

---

### Navigating the Pre-Generated Map Using the Bot

1. **Launch the navigation stack:**

```bash
source install/setup.bash
ros2 launch diff_car_model nav.launch.py
```

2. **Set the initial pose:**

   - Open RViz.
   - Use the `2D Pose Estimate` button to set the robot's initial pose on the map (this updates the `/initial_pose` topic).

3. **Set a goal pose:**

   - Use the `Nav2 Goal` button in RViz to set a goal position (this updates the `/goal_pose` topic).

The robot will navigate to the goal while avoiding both static and dynamic obstacles.

---

### Working of the stack

The bot operates using the following key functionalities:

#### 1. Sensor Fusion

The robot integrates data from multiple sensors to refine odometry and reduce drift. Specifically:

- **Sensors used**:

  - 2D LiDAR
  - Wheel encoders
  - Base IMU

- **Tool used**: `robot_localization`
- **Config file**: Located in `config/ekf.yaml`
- **How it works**:

  - Wheel encoder data provides relative displacement information.
  - IMU provides orientation and angular velocity.
  - The Extended Kalman Filter (EKF) in `robot_localization` fuses this data to produce accurate and drift-minimized odometry.

#### 2. Localization

Localization is handled by `nav2_amcl`:

- **How it works**:

  - The Adaptive Monte Carlo Localization (AMCL) algorithm uses the LiDAR scan and pre-generated map to localize the robot.
  - It continually updates the robot's position in the map frame by comparing the scan data to the map.

#### 3. Path Planning

Path planning is performed by the Nav2 stack:

- **Planner**:

  - A global planner generates the optimal path from the robot’s current position to the goal pose.
  - A local planner refines this path in real-time to account for dynamic obstacles.

- **Config file**: Located in `config/nav2_params.yaml`

#### 4. Obstacle Avoidance

The robot uses `nav2_collision_monitor` to avoid obstacles:

- **How it works:**

  - Uses user-defined polygon around the bot and other areas of interest to look for obstacles inside the polygon and command the robot to either stop or slow down
  - Polygons are also used to navigate around obstacles when the robot is stuck in a path and faces risk of colliding to an obstacle.
