"""Bring-up for the OAK-D Lite color point-cloud service.

Version-compatibility caveat: this launches Luxonis's depthai_ros_driver (its
"camera.launch.py") to drive the OAK-D Lite itself; lerobot_perception's own service node
then subscribes to that driver's RGB image / depth image / camera_info topics (see
config/oak_d_point_cloud_service_params.yaml). depthai_ros_driver's exact topic names and
the parameter that enables RGB/depth alignment (needed so every depth pixel lines up with
the same pixel in the color image - color_point_cloud_service_node requires matching image
sizes and will return a service error otherwise) have both changed across driver releases,
and were not verified against an installed copy in the environment this was written in.
Before trusting this out of the box, check, with the driver running:
  - `ros2 topic list | grep oak` against rgb_topic/depth_topic/camera_info_topic in
    config/oak_d_point_cloud_service_params.yaml
  - whichever parameter your installed depthai_ros_driver version currently uses to enable
    RGB/depth alignment (its own README/params file), and pass it via extra launch
    arguments to camera_driver_launch below if it isn't already the default

Set launch_camera_driver:=false to bring the camera driver up yourself (however your
installed version requires) and only launch lerobot_perception's service node here.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    launch_camera_driver_arg = DeclareLaunchArgument("launch_camera_driver", default_value="true")
    launch_camera_driver = LaunchConfiguration("launch_camera_driver")

    service_params = os.path.join(
        get_package_share_directory("lerobot_perception"),
        "config",
        "oak_d_point_cloud_service_params.yaml",
    )

    camera_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("depthai_ros_driver"), "launch", "camera.launch.py"
            )
        ),
        condition=IfCondition(launch_camera_driver),
    )

    color_point_cloud_service_node = Node(
        package="lerobot_perception",
        executable="color_point_cloud_service_node",
        # Name must match the "color_point_cloud_service:" top-level key in
        # config/oak_d_point_cloud_service_params.yaml.
        name="color_point_cloud_service",
        output="screen",
        parameters=[service_params],
    )

    return LaunchDescription(
        [
            launch_camera_driver_arg,
            camera_driver_launch,
            color_point_cloud_service_node,
        ]
    )
