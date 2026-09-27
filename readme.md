# ROS 2 Package for LeRobot SO-ARM101

LeRobot SO-ARM101 integrated into ROS 2 Jazzy.

## Features

- ✅ ROS 2 Jazzy compatibility
- ✅ Rviz visualization
- ✅ Gazebo Harmonic simulation
- ✅ ROS 2 Control integration
- ✅ MoveIt 2 motion planning
- ✅ Task-space control of the physical arm (MoveIt 2 IK/planning + a Feetech STS3215 serial driver, see `lerobot_hardware`)
- ✅ Color point cloud service for an OAK-D Lite RGB-D camera, with optional voxel downsampling and PCL Region Growing RGB segmentation (see `lerobot_perception`)
- ✅ One-command bring-up of the physical arm and the camera together (see `lerobot_bringup`)
---
## Installation

Clone this repository and install dependencies using [rosdep](https://docs.ros.org/en/ros2_packages/rosdep.html):


### Clone the repository
`git clone https://github.com/Pavankv92/lerobot_ws.git`

`cd lerobot_ws`

### Install ROS 2 dependencies
`rosdep update`

`rosdep install --from-paths src --ignore-src -r -y`

### Build
`colcon build`

---
## Rviz

**Summary:** Visualising LeRobot SO101 in Rviz

**Command:**  
`ros2 launch lerobot_description so101_display.launch.py`

**Video:**  
<!-- Add your video link here -->
https://github.com/user-attachments/assets/98f0a867-46c5-4661-8308-5de9e60a960b

---

## Gazebo and ROS 2 Control

**Summary:** Gazebo and ROS 2 Control: Control the gripper

**Commands:**  
`ros2 launch lerobot_description so101_gazebo.launch.py`  
`ros2 launch lerobot_controller so101_controller.launch.py`

**Video:**  
<!-- Add your video link here -->


https://github.com/user-attachments/assets/7d82b15c-8276-43b1-9b73-00b3567a5cf7


---

## Gazebo, ROS 2 Control and MoveIt

**Summary:** Gazebo, ROS 2 Control and MoveIt 2: MoveIt planner for the arm and gripper

**Commands:**  
`ros2 launch lerobot_description so101_gazebo.launch.py`  
`ros2 launch lerobot_controller so101_controller.launch.py`  
`ros2 launch lerobot_moveit so101_moveit.launch.py`

**Settings:**
- select "ompl" planning library for "arm" and "gripper" groups 

**Video: Arm**  
<!-- Add your video link here -->


https://github.com/user-attachments/assets/f95e9fd7-272a-46a1-8b34-0cb6c3f36da8

**Video: Gripper**  
<!-- Add your video link here -->

https://github.com/user-attachments/assets/5511c329-faad-4020-9527-4034f54a027a

---

## Physical Hardware

**Summary:** Command the real SO-ARM101 in task space. `pose_commander_node` takes a 4x4
homogeneous transform, validates it, and runs it through MoveIt 2 (moveit_py) for IK and
motion planning; `hardware_driver_node` executes the resulting trajectory on the real
Feetech STS3215 bus servos and publishes `/joint_states` back to MoveIt.

**Prerequisites:**
- `pip install pyserial` (or the `python3-serial` apt package)
- Wire the arm's serial bus adapter and note its device path (e.g. `/dev/ttyACM0`)
- Calibrate `src/lerobot_hardware/config/hardware_params.yaml`: `serial_port`, `servo_ids`,
  `joint_directions`, and `joint_offsets_rad` so that 0 rad matches each servo's mounted
  zero position

**Command:**  
`ros2 launch lerobot_hardware so101_physical.launch.py`

**Sending a pose command** (row-major 4x4 identity transform, i.e. no rotation, at
x=0.2 y=0.0 z=0.2 m in the `base` frame):
```bash
ros2 action send_goal /move_to_pose lerobot_hardware_interfaces/action/MoveToPose \
  "{pose_matrix: [1,0,0,0.2, 0,1,0,0.0, 0,0,1,0.2, 0,0,0,1]}"
```

- 📝 **TODO:** record a demo video

---

## Perception: OAK-D Lite Color Point Cloud

**Summary:** `color_point_cloud_service_node` (C++, PCL) exposes a
`get_color_point_cloud` service (`lerobot_perception_interfaces/srv/GetColorPointCloud`).
On each request it builds an XYZRGB cloud from the OAK-D Lite's latest synchronized
color+depth frame, always in the camera's own optical frame (it never transforms into a
world/base frame - do that yourself downstream with tf2 if you need it there), then
optionally:
- voxel-grid downsamples it to a requested grid size in meters, and/or
- segments it with PCL's Color-Based Region Growing (`pcl::RegionGrowingRGB`), returning
  the cloud re-colored one solid color per cluster.

**Prerequisites:**
- Luxonis `depthai_ros_driver` installed and providing RGB/depth-aligned images (same
  resolution, same optical frame) - see the version-compatibility caveat at the top of
  `lerobot_perception/launch/oak_d_point_cloud_service.launch.py` before relying on the
  default topic names in `lerobot_perception/config/oak_d_point_cloud_service_params.yaml`
- `libpcl-all-dev` (PCL) and `libopencv-dev`/`ros-jazzy-cv-bridge`

**Command:**  
`ros2 launch lerobot_perception oak_d_point_cloud_service.launch.py`

**Calling the service** (downsample to a 1cm grid, then segment):
```bash
ros2 service call /get_color_point_cloud lerobot_perception_interfaces/srv/GetColorPointCloud \
  "{downsample: true, voxel_leaf_size_m: 0.01, segment: true}"
```

**Parallelized:**
- The depth→XYZRGB pixel loop (`buildCloud()`) runs across rows via OpenMP, each thread
  appending to its own `std::vector` (no fixed-size/organized/NaN-padded cloud) which are
  concatenated into the final cloud once every thread finishes.
- Concurrent `get_color_point_cloud` requests: the service uses a `Reentrant` callback
  group and `main()` spins with a `MultiThreadedExecutor`, so independent requests that
  arrive close together are handled on separate threads rather than queuing - safe because
  `handleRequest()` only ever touches its own local copies and stack-allocated PCL objects.

**Identified, not implemented:**
- The rgb/depth `cv_bridge` conversions are independent of each other and could run
  concurrently, though they're cheap relative to the pixel loop (the depth conversion in
  particular is already a zero-copy view, not real work).
- Voxel-grid downsampling's point→voxel-index assignment is embarrassingly parallel
  (PCL also ships GPU-accelerated variants); the reduction into voxel centroids is not.
- `RegionGrowingRGB`'s region-growing walk is inherently sequential (each seed's growth
  order affects the result), so it's the least parallelizable of the four; only its KdTree
  construction has an independent sub-step.

- 📝 **TODO:** record a demo video

---

## Bring-up: Arm + Camera Together

**Summary:** `lerobot_bringup` launches the physical arm (`lerobot_hardware`) and the
OAK-D Lite perception service (`lerobot_perception`) in one command. Either half can be
turned off independently.

**Command:**  
`ros2 launch lerobot_bringup so101_bringup.launch.py`

**Arm or camera only:**
```bash
ros2 launch lerobot_bringup so101_bringup.launch.py launch_camera:=false   # arm only
ros2 launch lerobot_bringup so101_bringup.launch.py launch_arm:=false      # camera only
```

---

## License

This project is based on [RobotStudio SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100) and adheres to their license.







