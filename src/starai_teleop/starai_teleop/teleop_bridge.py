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

        leader_ns = str(self.get_parameter("leader_namespace").value).strip("/")
        follower_ns = str(self.get_parameter("follower_namespace").value).strip("/")
        self._command_time_ms = int(self.get_parameter("command_time_ms").value)
        self._startup_ramp_time_ms = int(self.get_parameter("startup_ramp_time_ms").value)
        self._stale_timeout = float(self.get_parameter("stale_timeout").value)

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
            f"-> follower=/{follower_ns}/set_angle_topic"
        )

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
            servo_id.append(index)
            target_angle.append(_joint_state_to_servo_angle(index, position))
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
