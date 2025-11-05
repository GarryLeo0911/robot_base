# AutoSLAM - Autonomous SLAM System# AutoSLAM - Autonomous SLAM System# AutoSLAM - Autonomous SLAM System# AutoSLAM - Autonomous SLAM System



**Overview**

- ROS 2 Jazzy package for basic robotic vehicle control

- Simple motor control with keyboard teleop (WASD)**Overview**

- Minimal foundation for future expansion

- ROS 2 Jazzy package for basic robotic vehicle control

**Prerequisites**

- Robot hardware with PCA9685 PWM controller for motors- Simple motor control with keyboard teleop (WASD)**Overview****Overview**

- ROS 2 Jazzy installed

- I2C enabled on Raspberry Pi (if using Pi hardware)- Minimal foundation for future expansion



**Quick Start**- ROS 2 Jazzy package for autonomous robotic vehicle control- ROS 2 Jazzy package for autonomous SLAM with robotic vehicles

1. **Build the package:**

   ```bash**Prerequisites**

   cd your_ros2_workspace

   colcon build --packages-select autoslam- Robot hardware with PCA9685 PWM controller for motors- Basic motor control with keyboard teleop (WASD)- Drive motors and keyboard teleop control

   source install/setup.bash

   ```- ROS 2 Jazzy installed



2. **Run basic robot control:**- I2C enabled on Raspberry Pi (if using Pi hardware)- Foundation for future SLAM capabilities- Optional SLAM with OAK‑D using RTAB‑Map (point cloud mode)

   ```bash

   ros2 launch autoslam bringup.launch.py

   ```

**Quick Start**- Camera runs on the robot; mapping and visualization run on the laptop

3. **Control the robot:**

   - Use WASD keys to control the robot1. **Build the package:**

   - W/S: Forward/Backward

   - A/D: Turn Left/Right   ```bash**Prerequisites**

   - Space: Stop

   - Q: Quit teleop   cd your_ros2_workspace



**What You Should See**   colcon build --packages-select autoslam- Robot hardware with PCA9685 PWM controller for motors**Prerequisites**

- Motor node starts and listens to `cmd_vel` commands

- Teleop node allows keyboard control   source install/setup.bash

- Robot responds to WASD commands for movement (at reduced speed)

   ```- ROS 2 Jazzy installed- Robot (car/RPi): OAK‑D connected.

**Built‑In Nodes**

- `motor_node`: subscribes `cmd_vel` and drives motors via PCA9685

- `teleop_wasd`: publishes keyboard control to `cmd_vel`

2. **Run basic robot control:**- I2C enabled on Raspberry Pi (if using Pi hardware)- If you use ROS 2 only: prefer `depthai_ros_driver` (native ROS 2).

**Parameters**

- `motor_node.max_duty` (int, default 1000): Maximum PWM duty cycle for motors (reduced for safer speed)   ```bash



**Hardware Setup**   ros2 launch autoslam bringup.launch.py- If you want an oakd_pcloud-style interface in ROS 2: use our ROS 2 wrapper `oakd_pcloud_ros2_compat.launch.py` which runs `depthai_ros_driver` and remaps topics to the oakd_pcloud names.

- Connect motors to PCA9685 PWM controller

- Ensure I2C is enabled: `sudo raspi-config` → Interface Options → I2C → Enable   ```

- Install I2C tools: `sudo apt install i2c-tools python3-smbus`

- Test I2C connection: `i2cdetect -y 1` (should show device at 0x40)**Quick Start**- If you must run the original `oakd_pcloud` (ROS 1): you’ll need a ROS 1 environment and `ros1_bridge`.



**Speed Control**3. **Control the robot:**

- Default speed is set to 1000 (50% of maximum) for safety

- To adjust speed, modify the `max_duty` parameter in launch file or pass as argument:   - Use WASD keys to control the robot1. **Build the package:**- Laptop: `rtabmap_slam`, `rtabmap_odom`, and `rviz2` installed (ROS 2).

  ```bash

  ros2 launch autoslam bringup.launch.py max_duty:=500    # Even slower   - W/S: Forward/Backward

  ros2 launch autoslam bringup.launch.py max_duty:=1500   # Faster

  ```   - A/D: Turn Left/Right   ```bash



**Notes**   - Space: Stop

- Ensure I2C is enabled on the Pi

- This is a minimal foundation - additional features can be added later   - Q: Quit teleop   cd your_ros2_workspace**Network Setup (both machines)**

- Focus on simple, reliable WASD control

- Speed has been reduced to half for safer operation

**What You Should See**   colcon build --packages-select autoslam- Same LAN and time sync (NTP). Allow UDP multicast in firewall.

- Motor node starts and listens to `cmd_vel` commands

- Teleop node allows keyboard control   source install/setup.bash- Set ROS 2 environment:

- Robot responds to WASD commands for movement

   ```  - Linux/macOS: `export ROS_DOMAIN_ID=22; export ROS_LOCALHOST_ONLY=0; export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`

**Built‑In Nodes**

- `motor_node`: subscribes `cmd_vel` and drives motors via PCA9685  - Windows PowerShell: `$Env:ROS_DOMAIN_ID=22; $Env:ROS_LOCALHOST_ONLY=0; $Env:RMW_IMPLEMENTATION='rmw_cyclonedds_cpp'`

- `teleop_wasd`: publishes keyboard control to `cmd_vel`

2. **Run basic robot control:**

**Parameters**

- `motor_node.max_duty` (int, default 2000): Maximum PWM duty cycle for motors   ```bash**Run On The Robot (camera + actuators)**



**Hardware Setup**   ros2 launch autoslam bringup.launch.py- Option A (oakd_pcloud-style in ROS 2):

- Connect motors to PCA9685 PWM controller

- Ensure I2C is enabled: `sudo raspi-config` → Interface Options → I2C → Enable   ```  - `ros2 launch autoslam oakd_pcloud_ros2_compat.launch.py`

- Install I2C tools: `sudo apt install i2c-tools python3-smbus`

- Test I2C connection: `i2cdetect -y 1` (should show device at 0x40)  - Requires `depthai_ros_driver` and `topic_tools` installed.



**Notes**3. **Control the robot:**  - Publishes PointCloud2 on `/stereo_rgb_node/stereo/points` and rectified images/camera_info with oakd_pcloud-compatible names.

- Ensure I2C is enabled on the Pi

- This is a minimal foundation - additional features can be added later   - Use WASD keys to control the robot  - Ensure a TF from `base_link` to the camera frame exists. With DepthAI, the point cloud often uses `<camera_name>_right_camera_optical_frame` (e.g., `oak_right_camera_optical_frame`). Set `cam_child_frame` accordingly in `slam_bringup.launch.py` or publish a matching static transform.

- Focus on simple, reliable WASD control
   - W/S: Forward/Backward- Option B (pure ROS 2 camera, no oakd compatibility): use `depthai_ros_driver` via our bringup.

   - A/D: Turn Left/Right  - `ros2 launch autoslam slam_bringup.launch.py include_camera:=true camera_pkg:=depthai_ros_driver camera_launch:=camera.launch.py`

   - Space: Stop- Option C (original oakd_pcloud - ROS 1) with bridging (advanced):

   - Q: Quit teleop  1) On the robot (ROS 1 shell):

     - `roslaunch oakd_pcloud stereo_nodelet.launch camera_name:=oak`

**What You Should See**     - This publishes `/stereo_rgb_node/stereo/points` (sensor_msgs/PointCloud2) in ROS 1.

- Motor node starts and listens to `cmd_vel` commands  2) On the laptop, run `ros1_bridge` dynamic bridge so ROS 2 can see the point cloud:

- Teleop node allows keyboard control     - Ensure both ROS 1 and ROS 2 are installed on the laptop.

- Robot responds to WASD commands for movement     - Source both environments, then run: `ros2 run ros1_bridge dynamic_bridge --bridge-all-1to2-topics`

     - Verify the topic appears in ROS 2 with: `ros2 topic list | findstr stereo_rgb_node/stereo/points`

**Built‑In Nodes**  3) Keep robot control on ROS 2: run our bringup (motors, TF):

- `motor_node`: subscribes `cmd_vel` and drives motors via PCA9685     - `ros2 launch autoslam slam_bringup.launch.py`

- `teleop_wasd`: publishes keyboard control to `cmd_vel`  - Adjust static TF if needed (camera mount):

- `odom_integrator_node`: integrates `cmd_vel` to publish `odom` and TF `odom->base_link`     - `cam_parent_frame:=base_link cam_child_frame:=oak-d_frame`

     - `cam_x:=0.08 cam_y:=0.0 cam_z:=0.08 cam_roll:=0 cam_pitch:=0 cam_yaw:=0`

**Parameters**

- `motor_node.max_duty` (int, default 2000): Maximum PWM duty cycle for motors**Run On The Laptop (mapping + RViz)**

- `odom_integrator_node.{odom_frame, base_frame, publish_rate_hz}`: Odometry settings- Start RTAB‑Map in scan‑cloud mode, consuming the bridged point cloud:

  - `ros2 launch autoslam laptop_rtabmap_oakd.launch.py`

**Hardware Setup**  - Default `cloud_topic` is `/stereo_rgb_node/stereo/points` (oakd_pcloud). Override if you remap.

- Connect motors to PCA9685 PWM controller  - Tip: Discover topics: `ros2 topic list | findstr points`

- Ensure I2C is enabled: `sudo raspi-config` → Interface Options → I2C → Enable  - Optional: disable ICP odom if you already publish odom: `start_icp_odom:=false`

- Install I2C tools: `sudo apt install i2c-tools python3-smbus`- Start RViz with 3D map view:

- Test I2C connection: `i2cdetect -y 1` (should show device at 0x40)  - `ros2 launch autoslam laptop_viz.launch.py rvizconfig:=$((ros2 pkg prefix autoslam)/share/autoslam/rviz/oakd_viz.rviz)`



**Notes****What You Should See**

- Ensure SPI and I2C are enabled on the Pi- Robot TF and model.

- For better odometry, consider adding wheel encoders- RTAB‑Map’s 3D map on `/rtabmap/cloud_map` and obstacles on `/rtabmap/cloud_obstacles`.

- This is a foundation - camera and SLAM features can be added later- The map stays stable in `map` frame while odom may drift.

**Bandwidth Tips**
- Prefer compressed image transport over Wi‑Fi (when using images).
- Reduce camera resolution/FPS and adjust point cloud settings to lower throughput.

**Built‑In Nodes**
- `motor_node`: subscribes `cmd_vel` and drives motors.
- `teleop_wasd`: publishes keyboard control to `cmd_vel`.
- `odom_integrator_node`: integrates `cmd_vel` to publish `odom` and TF `odom->base_link`.

**Nav2 (optional)**
- Launch: `ros2 launch autoslam nav2.launch.py map:=/path/to/map.yaml`
- Do not run `teleop_wasd` while Nav2 is active.

**Parameters (common)**
- `motor_node.max_duty` (int, default 2000)
- `odom_integrator_node.{odom_frame, base_frame, publish_rate_hz}`

**Notes**
- Ensure SPI, I2C, and camera are enabled on the Pi.
- For better odometry, replace the integrator with encoders + `robot_localization`.
