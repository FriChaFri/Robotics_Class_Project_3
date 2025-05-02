import numpy as np
from ForwardKinematics import robot
from InverseKinematics import get_q_given_pose, get_given_pose


all_q_options = get_q_given_pose(robot)
given_pose = get_given_pose() # this is the given pose in question 1 pulled from the InverseKinematics.py file

for i, q in enumerate(all_q_options):

    print(f"Current q {i+1}st:\n", np.round(q, 5))
    # First get the end-effector pose 
    # second spatial Jacobian at that config
    J = robot.jacob0(q, T = given_pose) 

    # Third desired twist [vx,vy,vz,wx,wy,wz]ᵀ
    v = np.array([100, -200, -300,   # linear velocity
                2,  -1,   0.5]) # angular velocity

    # Fourth solve for qdot
    qdot = np.linalg.inv(J) @ v


    # Fifth check
    err = np.linalg.norm(J @ qdot - v)
    print(f"\nResults in:\nq_dot = {qdot} {' ✅ Confirmed Correct' if err < 1e-6 else ' ❌ Not Correct'}\n")
    # print("reconstruction error:", err)
