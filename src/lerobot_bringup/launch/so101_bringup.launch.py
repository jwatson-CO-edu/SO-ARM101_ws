"""Full bring-up for the physical SO-ARM101: the arm and the OAK-D Lite perception
service, together.

Includes:
  - lerobot_hardware/launch/so101_physical.launch.py: robot_state_publisher, MoveIt 2
    (move_group + RViz), hardware_driver_node (drives the real Feetech servos),
    pose_commander_node (the "send it a 4x4 pose" MoveToPose action).
  - lerobot_perception/launch/oak_d_point_cloud_service.launch.py: depthai_ros_driver
    and color_point_cloud_service_node (the get_color_point_cloud service).

Either half can be turned off independently - e.g. to bench-test the arm before the
camera is wired up, or the camera before the arm is - via launch_arm / launch_camera.
See the two included launch files' own docstrings for their prerequisites and
version-compatibility caveats (in particular, oak_d_point_cloud_service.launch.py's note
on depthai_ros_driver's topic/parameter names).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    launch_arm_arg = DeclareLaunchArgument("launch_arm", default_value="true")
    launch_camera_arg = DeclareLaunchArgument("launch_camera", default_value="true")
    # Forwarded to oak_d_point_cloud_service.launch.py - set false to run
    # color_point_cloud_service_node against a camera driver you started yourself.
    launch_camera_driver_arg = DeclareLaunchArgument("launch_camera_driver", default_value="true")

    launch_arm = LaunchConfiguration("launch_arm")
    launch_camera = LaunchConfiguration("launch_camera")
    launch_camera_driver = LaunchConfiguration("launch_camera_driver")

    arm_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("lerobot_hardware"),
                "launch",
                "so101_physical.launch.py",
            )
        ),
        condition=IfCondition(launch_arm),
    )

    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("lerobot_perception"),
                "launch",
                "oak_d_point_cloud_service.launch.py",
            )
        ),
        launch_arguments={"launch_camera_driver": launch_camera_driver}.items(),
        condition=IfCondition(launch_camera),
    )

    return LaunchDescription(
        [
            launch_arm_arg,
            launch_camera_arg,
            launch_camera_driver_arg,
            arm_launch,
            camera_launch,
        ]
    )
