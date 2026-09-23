#!/usr/bin/env python
"""One-shot: send an arm to its named rest/park pose (see config/rest_poses.yaml).

This is a joint-space target, NOT the servo/URDF zero position -- see the
comment at the top of rest_poses.yaml for why those are different poses.
"""
from __future__ import annotations

import sys

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node

from robo_interfaces.msg import SetAngle

_ARM_JOINTS = ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6")


class GoToRest(Node):
    def __init__(self) -> None:
        super().__init__("starai_go_to_rest")
        self.declare_parameter("arm", "viola")
        self.declare_parameter("time_ms", 3000)
        self.declare_parameter(
            "config_path",
            f"{get_package_share_directory('starai_teleop')}/config/rest_poses.yaml",
        )

        arm = str(self.get_parameter("arm").value)
        self.time_ms = int(self.get_parameter("time_ms").value)
        config_path = str(self.get_parameter("config_path").value)

        with open(config_path) as f:
            poses = yaml.safe_load(f)
        if arm not in poses:
            raise ValueError(f"no rest pose defined for arm '{arm}' in {config_path}")
        self.target_deg = poses[arm]

        self.topic = f"/{arm}/set_angle_topic"
        self.pub = self.create_publisher(SetAngle, self.topic, 10)

    def send(self) -> None:
        self.get_logger().info(f"waiting for a subscriber on {self.topic} ...")
        while rclpy.ok() and self.count_subscribers(self.topic) == 0:
            rclpy.spin_once(self, timeout_sec=0.1)

        cmd = SetAngle()
        cmd.servo_id = list(range(6))
        cmd.target_angle = [float(self.target_deg[name]) for name in _ARM_JOINTS]
        cmd.time = [self.time_ms] * 6
        self.get_logger().info(f"sending rest pose over {self.time_ms} ms: {cmd.target_angle}")
        self.pub.publish(cmd)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GoToRest()
    try:
        node.send()
    except Exception as exc:  # surface config/arm errors clearly, then exit
        node.get_logger().error(str(exc))
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
