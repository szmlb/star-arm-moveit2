#!/usr/bin/env python
"""ROS2 bridge that mirrors a StarAI leader arm's live joint states onto a follower arm.

Subscribes to the leader's `joint_states` (published by `robo_driver`, radians for
joints 1-6, meters for the gripper) and republishes them as `robo_interfaces/SetAngle`
commands (degrees) on the follower's `set_angle_topic`, reusing the same
unit-conversion formulas as upstream's `viola_controller.py`.

After any gap longer than `stale_timeout` in leader updates (including the very
first message received), the next command is sent with a longer `startup_ramp_time_ms`
duration instead of the normal fast `command_time_ms`, so the follower eases toward
the leader's current pose instead of snapping to it if the two arms weren't
posed to match when tracking (re)starts.
"""
from __future__ import annotations

import math
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState

from robo_interfaces.msg import SetAngle

_JOINTS = ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7_left")
_JOINT_INDEX = {name: index for index, name in enumerate(_JOINTS)}


def _radians_to_degrees(radians: float) -> float:
    return radians * (180.0 / math.pi)


def _meters_to_degrees(meters: float) -> float:
    return (meters / 0.032) * 100.0


def _joint_state_to_servo_angle(index: int, position: float) -> float:
    if index < 6:
        return _radians_to_degrees(position)
    return _meters_to_degrees(position) + 100.0


class TeleopBridge(Node):
    """Mirrors a leader arm's joint_states onto a follower arm's set_angle_topic."""

    def __init__(self) -> None:
        super().__init__("starai_teleop_bridge")

        self.declare_parameter("leader_namespace", "violin")
        self.declare_parameter("follower_namespace", "viola")
        self.declare_parameter("command_time_ms", 70)
        self.declare_parameter("startup_ramp_time_ms", 800)
        self.declare_parameter("stale_timeout", 0.5)
        # Gripper-only linear remap, expressed as [open_deg, closed_deg] pairs
        # (NOT [min_deg, max_deg] -- viola and violin were found to use
        # OPPOSITE directions: violin low=open/high=closed, viola
        # low=closed/high=open. A naive same-direction min/max remap
        # commanded viola to open further than fully open and silently did
        # nothing once it hit that limit. Using explicit open/closed
        # endpoints makes the interpolation direction-agnostic and correct
        # regardless of which arm's convention runs which way. Values are in
        # the same degree-equivalent units as SetAngle.target_angle for the
        # gripper servo (see _joint_state_to_servo_angle). Defaults are from
        # hand-held-at-each-extreme readings on 2026-09-23, not a careful
        # calibration -- adjust if the mapping feels off.
        # NOTE: a physical USB reconnect was observed to shift the LEADER's
        # calibration entirely (open/closed went from 163.8/209.0 to
        # 29.4/-7.3 after one unplug-replug -- follower's stayed put).
        # Re-measure the leader side after any reconnect/power cycle.
        self.declare_parameter("leader_gripper_open_closed_deg", [29.4, -7.3])
        self.declare_parameter("follower_gripper_open_closed_deg", [106.0, 2.7])

        leader_ns = str(self.get_parameter("leader_namespace").value).strip("/")
        follower_ns = str(self.get_parameter("follower_namespace").value).strip("/")
        self._command_time_ms = int(self.get_parameter("command_time_ms").value)
        self._startup_ramp_time_ms = int(self.get_parameter("startup_ramp_time_ms").value)
        self._stale_timeout = float(self.get_parameter("stale_timeout").value)
        self._leader_gripper_open_closed = tuple(
            float(v) for v in self.get_parameter("leader_gripper_open_closed_deg").value
        )
        self._follower_gripper_open_closed = tuple(
            float(v) for v in self.get_parameter("follower_gripper_open_closed_deg").value
        )

        self._last_leader_time = 0.0

        self._command_publisher = self.create_publisher(
            SetAngle, f"/{follower_ns}/set_angle_topic", 10
        )
        self._leader_subscription = self.create_subscription(
            JointState,
            f"/{leader_ns}/joint_states",
            self._on_leader_joint_state,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            f"starai_teleop_bridge ready: leader=/{leader_ns}/joint_states "
            f"-> follower=/{follower_ns}/set_angle_topic "
            f"(gripper open/closed remap {self._leader_gripper_open_closed} "
            f"-> {self._follower_gripper_open_closed})"
        )

    def _remap_gripper_degrees(self, leader_deg: float) -> float:
        open_l, closed_l = self._leader_gripper_open_closed
        open_f, closed_f = self._follower_gripper_open_closed
        fraction_closed = (leader_deg - open_l) / (closed_l - open_l)
        fraction_closed = max(0.0, min(1.0, fraction_closed))
        return open_f + fraction_closed * (closed_f - open_f)

    def _on_leader_joint_state(self, msg: JointState) -> None:
        now = time.monotonic()
        resuming_after_gap = (now - self._last_leader_time) > self._stale_timeout
        self._last_leader_time = now
        time_ms = self._startup_ramp_time_ms if resuming_after_gap else self._command_time_ms

        servo_id = []
        target_angle = []
        command_time = []
        for name, position in zip(msg.name, msg.position):
            index = _JOINT_INDEX.get(name)
            if index is None or not math.isfinite(position):
                continue
            angle_deg = _joint_state_to_servo_angle(index, position)
            if index == 6:
                remapped = self._remap_gripper_degrees(angle_deg)
                self.get_logger().debug(
                    f"gripper: leader_pos={position:.5f} leader_deg={angle_deg:.2f} "
                    f"-> follower_deg={remapped:.2f}",
                    throttle_duration_sec=0.5,
                )
                angle_deg = remapped
            servo_id.append(index)
            target_angle.append(angle_deg)
            command_time.append(time_ms)

        if not servo_id:
            return

        command = SetAngle()
        command.servo_id = servo_id
        command.target_angle = target_angle
        command.time = command_time
        self._command_publisher.publish(command)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TeleopBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
