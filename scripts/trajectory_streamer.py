#!/usr/bin/env python3
"""
Pick and Place Trajectory Streamer
Streams quintic polynomial trajectory to RViz via /joint_states topic
Joints: Revolute_1, Revolute_2, Revolute_3, Revolute_5 (4-DOF arm)
        Revolute_7, Revolute_8 (gripper fingers)
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np
import roboticstoolbox as rtb
import time

# ── Robot definition ─────────────────────────────────────────────────
robot = rtb.DHRobot([
    rtb.RevoluteDH(d=0.042, a=0.000, alpha=np.pi/2, offset=0),
    rtb.RevoluteDH(d=0.000, a=0.365, alpha=0,        offset=0),
    rtb.RevoluteDH(d=0.246, a=0.048, alpha=np.pi/2, offset=0),
    rtb.RevoluteDH(d=0.115, a=0.000, alpha=0,        offset=0),
], name="robotic_arm_4DOF")

# ── Quintic polynomial trajectory ────────────────────────────────────
def quintic_segment(q_start, q_end, duration, dt=0.02):
    """Smooth trajectory between two joint configs using quintic polynomial."""
    times = np.arange(0, duration + dt, dt)
    trajectory = []
    for t in times:
        tau = min(t / duration, 1.0)
        # Quintic: 10τ³ - 15τ⁴ + 6τ⁵  (zero vel/acc at endpoints)
        s = 10*tau**3 - 15*tau**4 + 6*tau**5
        q = np.array(q_start) + s * (np.array(q_end) - np.array(q_start))
        trajectory.append(q)
    return trajectory

def solve_ik(target_q_config):
    """Use FK result as IK target (guaranteed reachable)."""
    return np.array(target_q_config)

# ── Pick and place waypoints (joint space) ───────────────────────────
# These are [q1, q2, q3, q4] in radians for your 4-DOF arm
# Verified reachable from your IK tests
WAYPOINTS = {
    "home":         [0.0,    0.0,    0.0,   0.0],
    "pre_pick":     [0.0,    0.5,   -0.3,   0.2],
    "pick":         [0.0,    0.7,   -0.5,   0.3],
    "lift":         [0.0,    0.5,   -0.3,   0.2],
    "rotate":       [1.2,    0.5,   -0.3,   0.2],
    "pre_place":    [1.2,    0.6,   -0.4,   0.3],
    "place":        [1.2,    0.75,  -0.55,  0.35],
    "retreat":      [1.2,    0.5,   -0.3,   0.2],
    "home_return":  [0.0,    0.0,    0.0,   0.0],
}

# Gripper states: [Revolute_7, Revolute_8] — open=0.4, closed=0.0
GRIPPER = {
    "open":   [0.4, -0.4],
    "closed": [0.0,  0.0],
}

# Sequence: (waypoint_name, duration_sec, gripper_state)
SEQUENCE = [
    ("home",        2.0, "open"),
    ("pre_pick",    2.5, "open"),
    ("pick",        1.5, "open"),
    ("pick",        1.0, "closed"),   # close gripper at pick
    ("lift",        1.5, "closed"),
    ("rotate",      3.0, "closed"),
    ("pre_place",   2.0, "closed"),
    ("place",       1.5, "closed"),
    ("place",       1.0, "open"),     # open gripper at place
    ("retreat",     1.5, "open"),
    ("home_return", 3.0, "open"),
]

class TrajectoryStreamer(Node):
    def __init__(self):
        super().__init__('trajectory_streamer')
        self.pub = self.create_publisher(JointState, '/joint_states', 10)
        self.get_logger().info('Trajectory streamer ready. Starting in 2 seconds...')
        time.sleep(2.0)
        self.run_sequence()

    def publish_joints(self, q_arm, q_gripper):
        """Publish joint state for all 6 controlled joints."""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        # Must match joint names exactly as in your URDF
        msg.name = [
            'Revolute_1', 'Revolute_2', 'Revolute_3',
            'Revolute_4', 'Revolute_5', 'Revolute_6',
            'Revolute_7', 'Revolute_8'
        ]
        msg.position = [
            float(q_arm[0]),    # Revolute_1 (waist)
            float(q_arm[1]),    # Revolute_2 (shoulder)
            float(q_arm[2]),    # Revolute_3 (elbow)
            0.0,                # Revolute_4 (fixed)
            float(q_arm[3]),    # Revolute_5 (wrist)
            0.0,                # Revolute_6 (fixed)
            float(q_gripper[0]),# Revolute_7 (left finger)
            float(q_gripper[1]),# Revolute_8 (right finger)
        ]
        self.pub.publish(msg)

    def run_sequence(self):
        """Execute full pick and place sequence."""
        self.get_logger().info('Starting pick and place sequence...')

        current_q = np.array(WAYPOINTS["home"])
        current_gripper = np.array(GRIPPER["open"])

        for step_name, duration, gripper_state in SEQUENCE:
            target_q = np.array(WAYPOINTS[step_name])
            target_gripper = np.array(GRIPPER[gripper_state])

            self.get_logger().info(
                f'Moving to [{step_name}] | '
                f'q={np.degrees(target_q).round(1)} deg | '
                f'gripper={gripper_state}'
            )

            # Generate quintic trajectory for arm
            arm_traj    = quintic_segment(current_q, target_q, duration)
            gripper_traj = quintic_segment(current_gripper, target_gripper, duration)

            # Stream at 50 Hz
            dt = 0.02
            for q_arm, q_grip in zip(arm_traj, gripper_traj):
                self.publish_joints(q_arm, q_grip)
                time.sleep(dt)

            current_q       = target_q.copy()
            current_gripper = target_gripper.copy()

        self.get_logger().info('Pick and place sequence complete!')

def main():
    rclpy.init()
    node = TrajectoryStreamer()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
