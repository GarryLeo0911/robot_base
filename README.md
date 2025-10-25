ROS 2 Jazzy package for Freenove 4WD Smart Car

Nodes
- motor_node: subscribes `cmd_vel` (geometry_msgs/Twist) and drives motors.
- led_node: subscribes `led_color` (std_msgs/ColorRGBA) to set all LEDs.
- ultrasonic_node: publishes `range` (sensor_msgs/Range) at given rate.
- camera_node: publishes `image_raw` (sensor_msgs/Image) using Picamera2.
- odom_integrator_node: open-loop odometry by integrating `cmd_vel` to publish `odom` and TF `odom->base_link`.

Launch
`ros2 launch ros2_freenove_4wd bringup.launch.py`

SLAM (OAK‑D + RTAB‑Map)
- Purpose: Teleop the car while building a map and getting localization from RTAB‑Map. Nav2 is not required.
- Requirements: OAK‑D and a camera driver. Either:
  - depthai_ros (ros-jazzy-depthai-ros) with its pointcloud launch, or
  - oakd_pcloud (https://github.com/h3ct0r/oakd_pcloud) for direct PointCloud2 publishing.
- Start SLAM teleop bringup (robot side, actuator/sensors only):
  - `ros2 launch ros2_freenove_4wd slam_bringup.launch.py`
  - Optional args:
    - `camera_pkg:=<your_camera_pkg>` (default `depthai_ros_driver`)
    - `camera_launch:=camera.launch.py` (camera-only launch under the camera package)
    - `include_camera:=true` (include the camera-only launch on the robot; no SLAM or heavy processing runs on the Pi)
    - `cam_parent_frame:=base_link` (parent frame)
    - `cam_child_frame:=oak-d_frame` (OAK‑D frame published by DepthAI)
    - `cam_x:=0.08 cam_y:=0.0 cam_z:=0.08 cam_roll:=0 cam_pitch:=0 cam_yaw:=0` (static TF from parent to camera)
- Notes:
  - This launch includes `motor_node`, `teleop_wasd`, `robot_state_publisher`, a static TF from `base_link` to the OAK‑D frame, and (optionally) includes a camera‑only launch from your camera package. It does not run RTAB‑Map on the Pi.
  - If your DepthAI (or other camera) package/launch path differs, pass the args above, or set them permanently by editing `launch/slam_bringup.launch.py`.
  - Make sure RTAB‑Map is configured to use `base_link` as the base frame and to publish `map->odom` TF. Visual odometry typically publishes `odom->base_link`.

Two‑Machine Setup (Robot streams, Laptop maps)
- Goal: Run sensors + motors on the Raspberry Pi (robot) and run RTAB‑Map visualization and mapping on your laptop over Wi‑Fi.

1) Network and ROS 2 setup
- Put both machines on the same LAN. Ensure clocks are synced (NTP).
- On both machines set the same domain and allow network discovery:
  - Linux/macOS: `export ROS_DOMAIN_ID=22; export ROS_LOCALHOST_ONLY=0`
  - Windows PowerShell: `$Env:ROS_DOMAIN_ID = 22; $Env:ROS_LOCALHOST_ONLY = 0`
- Use the same RMW on both. CycloneDDS is recommended:
  - Linux/macOS: `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`
  - Windows: `$Env:RMW_IMPLEMENTATION = 'rmw_cyclonedds_cpp'`
- Make sure firewalls allow UDP multicast on the LAN or temporarily disable firewall on the private network profile.

2) Start the robot (Pi)
- Run the bringup so the Pi publishes sensor topics but does not run RTAB‑Map:
  - Example with depthai_ros (adjust launch file):
    `ros2 launch ros2_freenove_4wd slam_bringup.launch.py include_camera:=true camera_pkg:=depthai_ros_driver camera_launch:=camera.launch.py`
  - Example with oakd_pcloud included directly:
    `ros2 launch ros2_freenove_4wd oakd_camera.launch.py camera_launch:=oakd_pcloud.launch.py`

Using oakd_pcloud with RTAB‑Map (scan-cloud mode)
- On the laptop, run RTAB‑Map to consume the 3D cloud from oakd_pcloud:
  - `ros2 launch ros2_freenove_4wd laptop_rtabmap_oakd.launch.py cloud_topic:=/oak/stereo/points`
  - Adjust `cloud_topic` to match the topic published by oakd_pcloud (use `ros2 topic list | findstr pointcloud` to discover).
  - Optionally disable ICP odometry: add `start_icp_odom:=false` if you already have odom.


3) Visualize on the laptop (RViz only)
- Start RViz with a ready-made config (edit topics inside RViz if your camera namespace differs):
  - `ros2 launch ros2_freenove_4wd laptop_viz.launch.py`
- The default config shows TF, RobotModel, and two Image panels subscribing to `/oak/rgb/image_raw` and `/oak/depth/image_raw` with compressed transports.
- If your DepthAI depth topic differs (e.g., `/oak/stereo/image_raw` or `/stereo/depth`), change the topic in RViz or provide your own config via `rvizconfig:=/path/to/your.rviz`.

Bandwidth Tips
- Prefer compressed image transport over Wi‑Fi. Leave `image_transport:=compressed` on laptop, and if available, enable compressed publishers or H264/H265 on the Pi side.
- Reduce camera resolution/FPS and depth confidence to lower throughput.
- If discovery across subnets is problematic, consider a ROS 2 Discovery Server or a domain bridge.

Navigation (Nav2)
- Purpose: Run Nav2 to plan/control and publish `cmd_vel` to the car.
- Prereqs: `nav2_bringup` installed on your system; a map YAML for localization, or run SLAM to create one.
- URDF: installed at `share/ros2_freenove_4wd/urdf/freenove_4wd.urdf` with frames `base_footprint`, `base_link`, `ultrasonic`, `camera`.
- Odom: `odom_integrator_node` provides open-loop odom (drifts). Replace with encoder-based odom + `robot_localization` when available.

Run Nav2
- Start motors on the robot: `ros2 run ros2_freenove_4wd motor_node`
- Launch Nav2 (loads URDF, odom, and Nav2): `ros2 launch ros2_freenove_4wd nav2.launch.py map:=/path/to/map.yaml`
- Optional RViz: `ros2 launch nav2_bringup rviz_launch.py`
- Note: Do not run `teleop_wasd` while Nav2 is active; both publish `cmd_vel`. If needed, add `twist_mux`.

Parameters
- motor_node: `max_duty` (int, default 2000).
- led_node: `count` (int, default 8), `bus` (int, default 0), `device` (int, default 0).
- ultrasonic_node: `trigger_pin` (int, default 27), `echo_pin` (int, default 22), `frame_id` (str, default `ultrasonic`), `rate_hz` (float, default 10.0).
- camera_node: `width` (int, default 640), `height` (int, default 480), `frame_id` (str, default `camera`), `fps` (int, default 15).
- odom_integrator_node: `odom_frame` (str, default `odom`), `base_frame` (str, default `base_link`), `publish_rate_hz` (float, default 50.0).
- Nav2 params: see `share/ros2_freenove_4wd/config/nav2_params.yaml`.

Notes
- Servo and buzzer are intentionally omitted.
- Ensure SPI, I2C, and camera are enabled on the Pi.
- Picamera2 is required for camera_node.
- For obstacle avoidance, add a lidar and enable obstacle layers in Nav2. With only ultrasonic, use static maps or consider a range-sensor layer if supported by your ROS 2 distro.
