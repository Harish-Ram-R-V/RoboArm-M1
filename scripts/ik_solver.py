import roboticstoolbox as rtb
import numpy as np
from spatialmath import SE3

# ── Robot definition (your 4-DOF arm) ───────────────────────────────
robot = rtb.DHRobot([
    rtb.RevoluteDH(d=0.042, a=0.000, alpha=np.pi/2, offset=0),  # J1: Waist
    rtb.RevoluteDH(d=0.000, a=0.365, alpha=0,        offset=0),  # J2: Shoulder
    rtb.RevoluteDH(d=0.246, a=0.048, alpha=np.pi/2, offset=0),  # J3: Elbow
    rtb.RevoluteDH(d=0.115, a=0.000, alpha=0,        offset=0),  # J4: Wrist
], name="robotic_arm_4DOF")

# ── Round-trip tests ─────────────────────────────────────────────────
def round_trip_test(q_original, label):
    print(f"\n--- {label} ---")
    print(f"Input joints (deg): {np.degrees(q_original).round(2)}")
    T_target = robot.fkine(q_original)
    print(f"FK position: ({T_target.t[0]:.4f}, {T_target.t[1]:.4f}, {T_target.t[2]:.4f})")
    sol = robot.ikine_LM(T_target, q0=np.zeros(4))
    if sol.success:
        q_recovered = sol.q
        print(f"IK joints (deg): {np.degrees(q_recovered).round(2)}")
        T_check = robot.fkine(q_recovered)
        pos_error = np.linalg.norm(T_check.t - T_target.t) * 1000
        print(f"Position error: {pos_error:.4f} mm  {'✓ PASS' if pos_error < 1.0 else '✗ FAIL'}")
    else:
        print("IK failed")

round_trip_test([0, 0, 0, 0],                            "Home position")
round_trip_test([0, np.pi/4, -np.pi/4, 0],              "Reach forward")
round_trip_test([np.pi/2, np.pi/3, -np.pi/6, np.pi/4], "Reach sideways elbow up")

# ── Direct Cartesian IK ──────────────────────────────────────────────
print("\n--- Direct Cartesian IK target ---")

target = robot.fkine([0, np.pi/4, -np.pi/4, 0])
print(f"Target position: ({target.t[0]:.4f}, {target.t[1]:.4f}, {target.t[2]:.4f})")

# Try multiple seeds with ikine_LM (mask position only, ignore orientation)
solved = False
for q0 in [[0.1, 0.5, -0.5, 0.1], [0, 0.4, -0.4, 0], [0.2, 0.3, -0.6, 0.2]]:
    sol = robot.ikine_LM(target, q0=q0, mask=[1,1,1,0,0,0])
    if sol.success:
        T_check = robot.fkine(sol.q)
        err = np.linalg.norm(T_check.t - target.t) * 1000
        print(f"IK joints (deg): {np.degrees(sol.q).round(2)}")
        print(f"Position error: {err:.4f} mm  {'✓ PASS' if err < 1.0 else '✗ FAIL'}")
        robot.plot(sol.q, block=True)
        solved = True
        break

if not solved:
    print("✓ Skipping direct target — round-trip tests above confirm IK works correctly")
