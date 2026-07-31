import roboticstoolbox as rtb
import numpy as np
from spatialmath import SE3

# ── Your 4-DOF arm DH table ─────────────────────────────────────────
# RevoluteDH(d, a, alpha, offset)
# Values extracted from your actual URDF joint origins

robot = rtb.DHRobot([
    rtb.RevoluteDH(d=0.042, a=0.000, alpha=np.pi/2,  offset=0),  # J1: Waist
    rtb.RevoluteDH(d=0.000, a=0.365, alpha=0,         offset=0),  # J2: Shoulder
    rtb.RevoluteDH(d=0.246, a=0.048, alpha=np.pi/2,  offset=0),  # J3: Elbow
    rtb.RevoluteDH(d=0.115, a=0.000, alpha=0,         offset=0),  # J4: Wrist (Rev_5)
], name="robotic_arm_4DOF")

print(robot)

# ── Test FK at home position (all zeros) ────────────────────────────
q_home = [0, 0, 0, 0]
T = robot.fkine(q_home)
print("\n--- FK at home (all joints = 0) ---")
print(T)
print(f"End-effector position: x={T.t[0]:.4f}, y={T.t[1]:.4f}, z={T.t[2]:.4f} m")

# ── Test a few configurations ────────────────────────────────────────
configs = {
    "Reach forward":    [0,      np.pi/4,  -np.pi/4, 0],
    "Reach sideways":   [np.pi/2, np.pi/4, -np.pi/4, 0],
    "Elbow up":         [0,      np.pi/3,  -np.pi/6, np.pi/4],
}

print("\n--- FK at different configurations ---")
for name, q in configs.items():
    T = robot.fkine(q)
    print(f"{name}: pos = ({T.t[0]:.3f}, {T.t[1]:.3f}, {T.t[2]:.3f}) m")

# ── Visualise ────────────────────────────────────────────────────────
print("\nOpening 3D plot... (close window to exit)")
robot.plot(q_home, block=True)
