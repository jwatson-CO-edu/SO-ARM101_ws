"""Task-space command node for the physical SO-ARM101.

Accepts a target end-effector pose as a flattened 4x4 homogeneous transform (the
MoveToPose action) and, if it is a valid rigid-body transform, hands it to MoveIt 2
(via moveit_py) for inverse kinematics and motion planning. If planning succeeds, the
resulting joint trajectory is executed through MoveIt's configured controllers
(arm_controller / gripper_controller), which are serviced by hardware_driver_node
against the real servos.

Embedding MoveItPy directly in this node (rather than only using the standalone move_group
node from lerobot_moveit/launch/so101_moveit.launch.py) is the standard ROS 2 way to script
motions in Python: MoveItPy stands up its own in-process planning pipeline, so this node
does not depend on move_group also being launched (though so101_physical.launch.py still
launches it too, purely so RViz has something to show the plan through).
"""

import os

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from moveit.planning import MoveItPy
from moveit_configs_utils import MoveItConfigsBuilder
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from lerobot_hardware.matrix_utils import (
    flat_pose_to_matrix,
    is_valid_homogeneous_transform,
    matrix_to_pose_stamped,
)
from lerobot_hardware_interfaces.action import MoveToPose


def _load_yaml(package_name: str, relative_path: str) -> dict:
    """Read a yaml file out of an installed package's share directory."""
    path = os.path.join(get_package_share_directory(package_name), relative_path)
    with open(path, "r") as f:
        return yaml.safe_load(f) or {}


class PoseCommanderNode(Node):
    def __init__(self) -> None:
        super().__init__("pose_commander_node")

        self.declare_parameter("planning_group", "arm")
        self.declare_parameter("pose_link", "gripper")
        # "base" and "world" are coincident (so101_base.xacro's base_joint has a zero
        # origin), so either works as the frame for incoming pose matrices; "base" is used
        # here since it's what a user commanding the arm actually thinks of as the origin.
        self.declare_parameter("base_frame", "base")
        self.declare_parameter("planning_time_s", 5.0)
        self.declare_parameter("max_velocity_scaling", 0.2)
        self.declare_parameter("max_acceleration_scaling", 0.2)

        self._planning_group = self.get_parameter("planning_group").value
        self._pose_link = self.get_parameter("pose_link").value
        self._base_frame = self.get_parameter("base_frame").value

        so101_urdf_path = os.path.join(
            get_package_share_directory("lerobot_description"), "urdf", "so101.urdf.xacro"
        )

        # Same builder chain as lerobot_moveit/launch/so101_moveit.launch.py, so this node's
        # view of the robot (URDF, SRDF groups, controller topology) always matches what
        # move_group would use. .planning_pipelines(pipelines=["ompl"]) pulls in
        # lerobot_moveit/config/ompl_planning.yaml, which supplies the "arm"/"gripper"
        # planner_configs both this node and move_group need to actually plan.
        moveit_config = (
            MoveItConfigsBuilder("so101", package_name="lerobot_moveit")
            .robot_description(file_path=so101_urdf_path)
            .robot_description_semantic(file_path="config/so101.srdf")
            .trajectory_execution(file_path="config/moveit_controllers.yaml")
            .planning_pipelines(pipelines=["ompl"])
            .to_moveit_configs()
        )

        # MoveItPy additionally needs a "plan_request_params" / "planning_scene_monitor_options"
        # block that MoveItConfigsBuilder doesn't produce (those are move_group-launch-only
        # concepts); moveit_py_params.yaml supplies them, following the moveit2_tutorials
        # motion_planning_python_api pattern. The three overrides below let planning_time_s /
        # max_velocity_scaling / max_acceleration_scaling be tuned per-launch instead of only
        # by editing that yaml file.
        moveit_py_config_dict = moveit_config.to_dict()
        moveit_py_config_dict.update(_load_yaml("lerobot_hardware", "config/moveit_py_params.yaml"))
        moveit_py_config_dict.setdefault("plan_request_params", {})
        moveit_py_config_dict["plan_request_params"]["planning_time"] = float(
            self.get_parameter("planning_time_s").value
        )
        moveit_py_config_dict["plan_request_params"]["max_velocity_scaling_factor"] = float(
            self.get_parameter("max_velocity_scaling").value
        )
        moveit_py_config_dict["plan_request_params"]["max_acceleration_scaling_factor"] = float(
            self.get_parameter("max_acceleration_scaling").value
        )

        self._moveit = MoveItPy(node_name="pose_commander_moveit_py", config_dict=moveit_py_config_dict)
        self._arm = self._moveit.get_planning_component(self._planning_group)

        cb_group = ReentrantCallbackGroup()
        self._action_server = ActionServer(
            self,
            MoveToPose,
            "move_to_pose",
            execute_callback=self._execute_move_to_pose,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=cb_group,
        )

        self.get_logger().info("pose_commander_node ready, waiting for MoveToPose goals")

    def _goal_callback(self, _goal_request) -> GoalResponse:
        return GoalResponse.ACCEPT

    def _cancel_callback(self, _goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    def _execute_move_to_pose(self, goal_handle):
        result = MoveToPose.Result()
        feedback = MoveToPose.Feedback()

        try:
            matrix = flat_pose_to_matrix(goal_handle.request.pose_matrix)
            valid, reason = is_valid_homogeneous_transform(matrix)
            if not valid:
                self.get_logger().warn(f"rejecting pose command: {reason}")
                goal_handle.abort()
                result.success = False
                result.message = f"invalid homogeneous transform: {reason}"
                return result

            feedback.phase = "planning"
            goal_handle.publish_feedback(feedback)

            pose_stamped = matrix_to_pose_stamped(
                matrix, frame_id=self._base_frame, stamp=self.get_clock().now().to_msg()
            )

            # This is the "try to use the existing MoveIt 2 motion planner for IK and motion
            # plan" step: set_goal_state with a Cartesian pose (rather than joint values)
            # makes MoveIt solve IK for `pose_link` internally as part of planning, so a
            # successful plan() here already implies a valid IK solution exists.
            self._arm.set_start_state_to_current_state()
            self._arm.set_goal_state(pose_stamped_msg=pose_stamped, pose_link=self._pose_link)
            plan_result = self._arm.plan()

            # plan_result overloads __bool__ to report overall planning success (per the
            # moveit_py API), so this covers both "no IK solution" and "IK solved but no
            # collision-free path" in one check.
            if not plan_result:
                self.get_logger().warn("MoveIt could not find a valid IK solution / motion plan")
                goal_handle.abort()
                result.success = False
                result.message = "no IK solution or motion plan found for the requested pose"
                return result

            # Catch a cancel that arrived while we were planning, before committing to a
            # (blocking, currently non-cancelable once started) execute() call below.
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                result.message = "canceled by client before execution started"
                return result

            feedback.phase = "executing"
            goal_handle.publish_feedback(feedback)

            # execute() dispatches the planned trajectory to whichever controller(s)
            # moveit_controllers.yaml names for these joints - arm_controller / gripper_
            # controller here - which hardware_driver_node services against the real servos.
            # This call blocks until the trajectory finishes, so a cancel requested mid-
            # motion is only noticed after execution completes (see note above).
            execution_outcome = self._moveit.execute(plan_result.trajectory, controllers=[])
            # moveit_py's execute() has returned different things (True/False, or None)
            # across MoveIt releases; treat only an explicit False as failure so this keeps
            # working regardless of which your install returns.
            execution_success = execution_outcome is not False

            if not execution_success:
                goal_handle.abort()
                result.success = False
                result.message = "trajectory execution failed"
                return result

            goal_handle.succeed()
            result.success = True
            result.message = "pose reached"
            return result

        except Exception as exc:  # noqa: BLE001
            # An uncaught exception here would leave this goal's execute_callback thread
            # dead without ever calling succeed()/abort()/canceled(): the action client
            # would then wait forever for a result that's never coming. Always resolve it.
            self.get_logger().error(f"move_to_pose failed: {exc}")
            if goal_handle.is_active:
                goal_handle.abort()
            result.success = False
            result.message = f"unhandled exception: {exc}"
            return result


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PoseCommanderNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
