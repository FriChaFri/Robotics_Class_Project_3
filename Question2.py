import numpy as np
from ForwardKinematics import robot
from InverseKinematics import get_q_given_pose


all_q_options = get_q_given_pose(robot)

for i, q in enumerate(all_q_options):
    print(f"q_option {i+1}:\n", q)
    # (1) question 1b joint angles
    q_current = all_q_options[0] 

    # (2) spatial Jacobian at that config
    J = robot.jacob0(q_current)    # shape (6,6) for a 6-DOF

    # (3) desired twist [vx,vy,vz,wx,wy,wz]ᵀ
    v = np.array([10, -20, -30,   # linear velocity
                2,  -1,   0.5]) # angular velocity

    # (4) solve for qdot

    qdot = np.linalg.inv(J) @ v


    # (5) check
    err = np.linalg.norm(J @ qdot - v)
    print(f"\nResults in:\nq_dot = {qdot} {' ✅ Confirmed Correct' if err < 1e-6 else ' ❌ Not Correct'}\n")
    # print("reconstruction error:", err)
