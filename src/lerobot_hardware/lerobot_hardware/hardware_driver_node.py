"""Physical joint-control node for the SO-ARM101.

Bridges MoveIt 2's trajectory execution to the real Feetech STS3215 bus servos.

How this fits into the MoveIt 2 pipeline: `lerobot_moveit/config/moveit_controllers.yaml`
tells moveit_simple_controller_manager that there are two FollowJointTrajectory-based
controllers, "arm_controller" (joints 1-5) and "gripper_controller" (joint 6), each
reachable at "<controller_name>/follow_joint_trajectory". Normally those action servers
would be provided by joint_trajectory_controller running under ros2_control. Here we
provide them ourselves and translate each trajectory point directly into servo commands.
This also means we publish /joint_states ourselves (normally joint_state_broadcaster's
job), since MoveIt/RViz need real feedback to know where the arm actually is.

This node stands in for ros2_control + a hardware plugin: it is a much smaller amount of
code to get a hobby-scale serial-bus arm moving, at the cost of not being a general,
reusable ros2_control hardware interface.
"""

import threading
import time
from dataclasses import dataclass
from typing import Dict, List

import rclpy
from builtin_interfaces.msg import Duration as DurationMsg
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

from lerobot_hardware.feetech_bus import FeetechBus, radians_to_ticks, ticks_to_radians

# Joint names must match the URDF/SRDF exactly - this repo names arm joints "1".."5" and
# the gripper joint "6" (see lerobot_description/urdf/so101_base.xacro), which is also how
# they're split between the two moveit_simple_controller_manager controllers below.
ARM_JOINTS = ["1", "2", "3", "4", "5"]
GRIPPER_JOINTS = ["6"]
ALL_JOINTS = ARM_JOINTS + GRIPPER_JOINTS


@dataclass
class JointConfig:
    """Per-joint mapping between URDF radians and a physical servo's raw tick counts."""

    name: str
    servo_id: int
    direction: int  # +1 or -1: flips sign if the servo horn is mounted "backwards"
    offset_rad: float  # added so that a servo's own zero tick lines up with the URDF's 0 rad
    min_rad: float
    max_rad: float


def _duration_to_seconds(duration: DurationMsg) -> float:
    """builtin_interfaces/Duration (sec + nanosec) -> float seconds."""
    return duration.sec + duration.nanosec * 1e-9


class HardwareDriverNode(Node):
    def __init__(self) -> None:
        super().__init__("so101_hardware_driver")

        # -- parameters ----------------------------------------------------------------------
        # All defaults below are overridden at launch time by
        # lerobot_hardware/config/hardware_params.yaml. Keep defaults here safe (dry_run-able)
        # so an accidental `ros2 run` without parameters can't drive real hardware unexpectedly.
        self.declare_parameter("serial_port", "/dev/ttyACM0")
        self.declare_parameter("baud_rate", 1_000_000)
        self.declare_parameter("state_publish_rate_hz", 30.0)
        self.declare_parameter("dry_run", False)
        self.declare_parameter("joint_names", ALL_JOINTS)
        self.declare_parameter("servo_ids", [1, 2, 3, 4, 5, 6])
        self.declare_parameter("joint_directions", [1, 1, 1, 1, 1, 1])
        self.declare_parameter("joint_offsets_rad", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        # These min/max defaults are copied from lerobot_description's ros2_control xacro,
        # so a dry run with no yaml override still clamps to the real joint limits.
        self.declare_parameter(
            "joint_min_rad", [-1.91986, -1.74533, -1.74533, -1.65806, -2.79253, -0.174533]
        )
        self.declare_parameter(
            "joint_max_rad", [1.91986, 1.74533, 1.5708, 1.65806, 2.79253, 1.74533]
        )
        self.declare_parameter("waypoint_poll_period_s", 0.02)

        joint_names = list(self.get_parameter("joint_names").value)
        servo_ids = list(self.get_parameter("servo_ids").value)
        directions = list(self.get_parameter("joint_directions").value)
        offsets = list(self.get_parameter("joint_offsets_rad").value)
        min_rads = list(self.get_parameter("joint_min_rad").value)
        max_rads = list(self.get_parameter("joint_max_rad").value)

        # ROS2 won't catch a length mismatch between these parallel parameter arrays for us
        # (they're just independent typed arrays as far as rclpy is concerned), so verify it
        # ourselves rather than silently zipping to the shortest list or index-erroring later.
        if not (
            len(joint_names)
            == len(servo_ids)
            == len(directions)
            == len(offsets)
            == len(min_rads)
            == len(max_rads)
        ):
            raise ValueError(
                "joint_names, servo_ids, joint_directions, joint_offsets_rad, "
                "joint_min_rad and joint_max_rad must all be the same length"
            )

        self._joints: Dict[str, JointConfig] = {
            name: JointConfig(name, servo_ids[i], directions[i], offsets[i], min_rads[i], max_rads[i])
            for i, name in enumerate(joint_names)
        }

        self._waypoint_poll_period_s = float(self.get_parameter("waypoint_poll_period_s").value)

        self._bus = FeetechBus(
            port=self.get_parameter("serial_port").value,
            baud_rate=int(self.get_parameter("baud_rate").value),
            dry_run=bool(self.get_parameter("dry_run").value),
        )

        # The Feetech bus is a single shared half-duplex serial line: only one instruction/
        # response exchange may be in flight at a time, or the byte streams interleave and
        # both sides get garbage. The state-publishing timer and the two action servers'
        # execute callbacks all run concurrently (MultiThreadedExecutor + a Reentrant
        # callback group, below), so every call into self._bus must hold this lock.
        self._bus_lock = threading.Lock()

        with self._bus_lock:
            for joint in self._joints.values():
                if not self._bus.ping(joint.servo_id):
                    self.get_logger().warn(
                        f"servo id {joint.servo_id} (joint '{joint.name}') did not respond to ping"
                    )
                self._bus.set_torque_enable(joint.servo_id, True)

        # Cache of the last-read position per joint, in radians. Guards against a transient
        # read error making /joint_states jump to zero, and doubles as the "actual" position
        # reported in FollowJointTrajectory feedback without touching the bus a second time.
        self._state_cache_lock = threading.Lock()
        self._last_positions_rad: Dict[str, float] = {name: 0.0 for name in self._joints}

        self._joint_state_pub = self.create_publisher(JointState, "joint_states", 10)
        rate_hz = float(self.get_parameter("state_publish_rate_hz").value)
        # A single ReentrantCallbackGroup lets the state timer keep firing (and lets a
        # cancel request on one action server get serviced) while the other action
        # server's execute callback is mid-trajectory. MultiThreadedExecutor is required
        # for this to actually run concurrently rather than just being "allowed to".
        cb_group = ReentrantCallbackGroup()
        self._state_timer = self.create_timer(1.0 / rate_hz, self._publish_joint_states, callback_group=cb_group)

        self._arm_action_server = ActionServer(
            self,
            FollowJointTrajectory,
            "arm_controller/follow_joint_trajectory",
            execute_callback=lambda gh: self._execute_trajectory(gh, ARM_JOINTS),
            goal_callback=lambda goal: self._goal_callback(goal, ARM_JOINTS),
            cancel_callback=self._cancel_callback,
            callback_group=cb_group,
        )
        self._gripper_action_server = ActionServer(
            self,
            FollowJointTrajectory,
            "gripper_controller/follow_joint_trajectory",
            execute_callback=lambda gh: self._execute_trajectory(gh, GRIPPER_JOINTS),
            goal_callback=lambda goal: self._goal_callback(goal, GRIPPER_JOINTS),
            cancel_callback=self._cancel_callback,
            callback_group=cb_group,
        )

        self.get_logger().info("so101_hardware_driver ready")

    # -- joint state -----------------------------------------------------------------------

    def _read_joint_rad(self, joint: JointConfig) -> float:
        with self._bus_lock:
            ticks = self._bus.read_position_ticks(joint.servo_id)
        return ticks_to_radians(ticks, joint.direction, joint.offset_rad)

    def _publish_joint_states(self) -> None:
        """Timer callback: poll every servo's present position and publish /joint_states.

        This is the only feedback path MoveIt has into the real robot's state, so a joint
        missing from this message (or stuck at a stale value) will make planning start from
        the wrong current pose.
        """
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        names: List[str] = []
        positions: List[float] = []
        for name, joint in self._joints.items():
            try:
                rad = self._read_joint_rad(joint)
            except Exception as exc:  # noqa: BLE001 - one bad read shouldn't drop the topic
                self.get_logger().warn(f"failed to read joint '{name}': {exc}")
                with self._state_cache_lock:
                    rad = self._last_positions_rad[name]
            names.append(name)
            positions.append(rad)
            with self._state_cache_lock:
                self._last_positions_rad[name] = rad
        msg.name = names
        msg.position = positions
        self._joint_state_pub.publish(msg)

    # -- action server callbacks -------------------------------------------------------------

    def _goal_callback(self, goal_request, expected_joints: List[str]) -> GoalResponse:
        """Reject a trajectory up front if it names a joint this controller doesn't own.

        moveit_simple_controller_manager only ever sends "arm_controller" its 5 arm joints
        and "gripper_controller" its 1 gripper joint (per moveit_controllers.yaml), so this
        should never actually reject anything in normal operation - it's a guard against a
        misconfigured client, not something the happy path relies on.
        """
        requested = set(goal_request.trajectory.joint_names)
        if not requested.issubset(set(expected_joints)):
            self.get_logger().warn(
                f"rejecting trajectory goal: joints {sorted(requested)} are not a subset of "
                f"{expected_joints}"
            )
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_callback(self, _goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    def _execute_trajectory(self, goal_handle, expected_joints: List[str]):
        """Stream a JointTrajectory to the servos, one waypoint at a time, on schedule.

        MoveIt already time-parameterizes the trajectory (see the
        AddTimeOptimalParameterization adapter in lerobot_moveit/config/ompl_planning.yaml),
        so simply sending each waypoint at its `time_from_start` reproduces the smooth,
        velocity/acceleration-limited motion MoveIt planned - no extra interpolation needed
        here.
        """
        result = FollowJointTrajectory.Result()
        try:
            trajectory = goal_handle.request.trajectory
            joint_names = list(trajectory.joint_names)

            for name in joint_names:
                if name not in self._joints:
                    goal_handle.abort()
                    result.error_code = FollowJointTrajectory.Result.INVALID_JOINTS
                    result.error_string = f"unknown joint '{name}'"
                    return result

            start_time = time.monotonic()
            feedback = FollowJointTrajectory.Feedback()

            for point in trajectory.points:
                target_elapsed = _duration_to_seconds(point.time_from_start)

                # Sleep in small slices (instead of one long time.sleep) so a cancel
                # request lands within one poll period instead of at the next waypoint.
                while True:
                    if goal_handle.is_cancel_requested:
                        goal_handle.canceled()
                        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                        result.error_string = "canceled by client"
                        return result
                    remaining = target_elapsed - (time.monotonic() - start_time)
                    if remaining <= 0.0:
                        break
                    time.sleep(min(self._waypoint_poll_period_s, remaining))

                ticks_by_id: Dict[int, int] = {}
                for name, position in zip(joint_names, point.positions):
                    joint = self._joints[name]
                    # Defense in depth: MoveIt should already respect joint_limits.yaml, but
                    # this is the last line of code between a ROS message and a real motor,
                    # so clamp rather than trust.
                    clamped = max(joint.min_rad, min(joint.max_rad, position))
                    if abs(clamped - position) > 1e-6:
                        self.get_logger().warn(
                            f"joint '{name}' target {position:.4f} rad outside "
                            f"[{joint.min_rad:.4f}, {joint.max_rad:.4f}], clamping"
                        )
                    ticks_by_id[joint.servo_id] = radians_to_ticks(
                        clamped, joint.direction, joint.offset_rad
                    )

                with self._bus_lock:
                    self._bus.sync_write_position_ticks(ticks_by_id)

                feedback.header.stamp = self.get_clock().now().to_msg()
                feedback.joint_names = joint_names
                feedback.desired = point
                with self._state_cache_lock:
                    feedback.actual = JointTrajectoryPoint(
                        positions=[self._last_positions_rad[name] for name in joint_names]
                    )
                goal_handle.publish_feedback(feedback)

            goal_handle.succeed()
            result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
            return result

        except Exception as exc:  # noqa: BLE001
            # Without this, a serial error mid-trajectory would raise out of the executor's
            # worker thread and the action client (MoveIt) would wait forever: no result is
            # ever sent for a goal whose execute_callback never returns. Always resolve the
            # goal, even on an unexpected failure.
            self.get_logger().error(f"trajectory execution failed: {exc}")
            if goal_handle.is_active:
                goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = f"unhandled exception: {exc}"
            return result

    def destroy_node(self) -> bool:
        self._bus.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HardwareDriverNode()
    # MultiThreadedExecutor is required, not just an optimization: the ReentrantCallbackGroup
    # above only allows concurrent callbacks, it doesn't create the threads to run them on.
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
