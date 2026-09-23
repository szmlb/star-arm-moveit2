#!/usr/bin/env python3
"""Move viola's TCP (link6 frame) by a fixed offset in world Z, via numerical IK.

See docs/starai-arm-ros2.md ("TCPをIKで動かす例") for the full walkthrough.

Reads live /viola/joint_states (requires robo_driver running with lock:=enable
on /viola), solves damped-least-squares position-only IK (orientation left
free -- fixing full 6-DOF pose at this near-home config is over-constrained
and fails to converge near a singularity) for the 6 arm joints, checks the
result against URDF joint limits and a max per-joint delta sanity cap, then
publishes one robo_interfaces/SetAngle command. Afterward reads joint_states
again and reports the actual TCP delta via forward kinematics, for comparison
against the commanded 0.1m target.
"""
import sys
import time

import numpy as np
import pinocchio as pin
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from robo_interfaces.msg import SetAngle

URDF = "/home/szmlb/workspace/ros/starai_ros2/src/viola_description/urdf/viola_description.urdf"
ARM_JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
JOINT_LIMITS_RAD = {
    "joint1": (-2.27, 2.27),
    "joint2": (-1.57, 1.57),
    "joint3": (-1.57, 1.57),
    "joint4": (-2.27, 2.27),
    "joint5": (-1.57, 1.57),
    "joint6": (-2.27, 2.27),
}
DZ_M = 0.1
MAX_JOINT_DELTA_DEG = 30.0
COMMAND_TIME_MS = 2500


def to_deg(rad):
    return rad * 180.0 / np.pi


class OneShot(Node):
    def __init__(self):
        super().__init__("viola_tcp_move")
        self.latest = None
        self.create_subscription(JointState, "/viola/joint_states", self._cb, 10)
        self.pub = self.create_publisher(SetAngle, "/viola/set_angle_topic", 10)

    def _cb(self, msg):
        self.latest = dict(zip(msg.name, msg.position))


def wait_for_state(node, timeout=5.0):
    start = time.time()
    while rclpy.ok() and time.time() - start < timeout:
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.latest is not None:
            return node.latest
    raise TimeoutError("no /viola/joint_states received within timeout")


def build_q(model, joint_positions):
    q = pin.neutral(model)
    for name in ARM_JOINTS:
        jid = model.getJointId(name)
        q[model.joints[jid].idx_q] = joint_positions[name]
    return q


def main():
    model = pin.buildModelFromUrdf(URDF)
    data = model.createData()
    frame_id = model.getFrameId("link6")
    arm_v_idx = [model.joints[model.getJointId(n)].idx_v for n in ARM_JOINTS]

    rclpy.init()
    node = OneShot()

    state = wait_for_state(node)
    print("current /viola/joint_states:", {k: round(v, 4) for k, v in state.items()})

    q = build_q(model, state)
    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacements(model, data)
    current_t = np.array(data.oMf[frame_id].translation)
    print("current link6 position (m):", current_t)

    # Position-only IK (orientation left free): fixing full 6-DOF orientation
    # at this near-home pose is over-constrained near a singularity and fails
    # to converge; a pure +Z position target is well-posed and matches what
    # was actually asked for (move TCP up, not "and keep this exact tilt").
    target_t = current_t + np.array([0.0, 0.0, DZ_M])
    print("target link6 position (m):", target_t)

    eps, IT_MAX, DT, damp = 1e-6, 2000, 0.3, 1e-6
    success = False
    err_norm = None
    for i in range(IT_MAX):
        pin.forwardKinematics(model, data, q)
        pin.updateFramePlacements(model, data)
        cur_t = np.array(data.oMf[frame_id].translation)
        err = target_t - cur_t
        err_norm = np.linalg.norm(err)
        if err_norm < eps:
            success = True
            break
        J = pin.computeFrameJacobian(model, data, q, frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
        J_pos = J[:3, arm_v_idx]
        v_arm = J_pos.T @ np.linalg.solve(J_pos @ J_pos.T + damp * np.eye(3), err)
        v = np.zeros(model.nv)
        for k, idx in enumerate(arm_v_idx):
            v[idx] = v_arm[k]
        q = pin.integrate(model, q, v * DT)

    print(f"IK {'converged' if success else 'DID NOT CONVERGE'} after {i + 1} iters, "
          f"residual={err_norm:.2e}")
    if not success:
        print("ABORT: IK did not converge, not sending anything.")
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    target_deg = {}
    for name in ARM_JOINTS:
        jid = model.getJointId(name)
        val = q[model.joints[jid].idx_q]
        lo, hi = JOINT_LIMITS_RAD[name]
        if not (lo <= val <= hi):
            print(f"ABORT: {name} target {val:.3f} rad is outside URDF limit [{lo}, {hi}]")
            node.destroy_node()
            rclpy.shutdown()
            sys.exit(1)
        target_deg[name] = to_deg(val)

    print("target joint angles (deg):", {k: round(v, 2) for k, v in target_deg.items()})

    for name in ARM_JOINTS:
        delta = abs(target_deg[name] - to_deg(state[name]))
        print(f"  {name}: delta {delta:.2f} deg")
        if delta > MAX_JOINT_DELTA_DEG:
            print(f"ABORT: {name} delta {delta:.1f} deg exceeds safety cap "
                  f"({MAX_JOINT_DELTA_DEG} deg), refusing to send.")
            node.destroy_node()
            rclpy.shutdown()
            sys.exit(1)

    cmd = SetAngle()
    cmd.servo_id = list(range(6))
    cmd.target_angle = [target_deg[n] for n in ARM_JOINTS]
    cmd.time = [COMMAND_TIME_MS] * 6
    print(f"Sending SetAngle over {COMMAND_TIME_MS} ms:", list(zip(cmd.servo_id, cmd.target_angle)))
    time.sleep(0.3)  # let the publisher match with robo_driver's subscription
    node.pub.publish(cmd)

    wait_s = COMMAND_TIME_MS / 1000.0 + 1.0
    print(f"Waiting {wait_s:.1f}s for motion to complete...")
    time.sleep(wait_s)

    node.latest = None
    for _ in range(20):
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.latest is not None:
            break
    state2 = wait_for_state(node)
    q2 = build_q(model, state2)
    pin.forwardKinematics(model, data, q2)
    pin.updateFramePlacements(model, data)
    result = np.array(data.oMf[frame_id].translation)
    print("resulting link6 position (m):", result)
    print("actual delta (m):", result - current_t)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
