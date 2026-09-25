"""Bring-up for controlling the *physical* SO-ARM101.

This intentionally does NOT include lerobot_controller/launch/so101_controller.launch.py:
that file's controller_manager + ros2_control spawners are wired to the
gz_ros2_control/GazeboSimSystem hardware plugin (see
lerobot_description/urdf/so101_ros2_control.xacro), which only exists in simulation. For
real hardware, lerobot_hardware's hardware_driver_node takes over that role directly against
the Feetech servos, so it - not ros2_control - is launched here alongside it.

Nodes started:
  - robot_state_publisher: publishes tf (world -> base -> ... -> gripper) from the URDF, so
    RViz (and anything else that wants link poses, not just joint angles) can render the
    real robot. so101_moveit.launch.py does not start this itself.
  - so101_moveit.launch.py (included, is_sim:=False): starts move_group + RViz, giving a
    visual/interactive view of planning: pose_commander_node still does its own planning
    through its own embedded MoveItPy instance, independent of this move_group process.
  - so101_hardware_driver: drives the real servos and publishes /joint_states.
  - pose_commander_node: the "send it a 4x4 pose" entry point (MoveToPose action).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    hardware_params = os.path.join(
        get_package_share_directory("lerobot_hardware"), "config", "hardware_params.yaml"
    )

    # Expand the xacro at launch time, the same way so101_controller.launch.py does for
    # Gazebo - robot_state_publisher needs the final URDF XML, not the xacro source.
    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                os.path.join(
                    get_package_share_directory("lerobot_description"),
                    "urdf",
                    "so101.urdf.xacro",
                ),
            ]
        ),
        value_type=str,
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": False}],
    )

    # is_sim:=False only changes the move_group node's use_sim_time parameter here (see
    # so101_moveit.launch.py) - it does not change which nodes get launched.
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("lerobot_moveit"), "launch", "so101_moveit.launch.py"
            )
        ),
        launch_arguments={"is_sim": "False"}.items(),
    )

    hardware_driver_node = Node(
        package="lerobot_hardware",
        executable="hardware_driver_node",
        # Name must match the "so101_hardware_driver:" top-level key in hardware_params.yaml
        # for those ros__parameters to actually be applied to this node.
        name="so101_hardware_driver",
        output="screen",
        parameters=[hardware_params],
    )

    pose_commander_node = Node(
        package="lerobot_hardware",
        executable="pose_commander_node",
        # Name must match the "pose_commander_node:" top-level key in hardware_params.yaml.
        name="pose_commander_node",
        output="screen",
        parameters=[hardware_params],
    )

    return LaunchDescription(
        [
            robot_state_publisher_node,
            moveit_launch,
            hardware_driver_node,
            pose_commander_node,
        ]
    )
