import numpy as np
from spatialmath.base import tr2rpy
from roboticstoolbox import DHRobot, RevoluteDH

links = [ # in millimeters and radians
    RevoluteDH(alpha=np.pi/2,  a=27.5, d=339,  offset=0      ),  # link 1
    RevoluteDH(alpha=0,        a= 250, d=  0,  offset=0),  # link 2 #offset np.pi/2
    RevoluteDH(alpha=np.pi/2,  a=  70, d=  0,  offset=0      ),  # link 3
    RevoluteDH(alpha=-np.pi/2, a=   0, d=250,  offset=0      ),  # link 4
    RevoluteDH(alpha=np.pi/2,  a=   0, d=  0,  offset=0      ),  # link 5
    RevoluteDH(alpha=0,        a=   0, d= 95,  offset=0      ),  # link 6
]

robot = DHRobot(links, name="miniBOT6-R")

if __name__ == "__main__":

    robot.q = [0, np.pi/2, 0, 0, -np.pi/2, 0]
    print("Home Pose:")
    home_pose = robot.fkine([0,0,0,0,0,0])
    print(home_pose) 
    print("Q1 Pose:") # with Theta2 = 90 degrees and Theta5 = -90 degrees:
    q1_pose = robot.fkine(robot.q-robot.offset)
    print(q1_pose)
    print("Combined position and Euler ZYX for Q1 Pose:")
    print(np.concatenate((q1_pose.t.round(5), tr2rpy(q1_pose.A, 'zyx').round(5))))

    # q_start = np.array([0, 0, 0, 0, 0, 0])
    # q_end   = np.array([np.pi/4, -np.pi/6, np.pi/3, 0, np.pi/2, -np.pi/4])
    # # (3) Build a time vector and linearly interpolate
    # steps = 100
    # t = np.linspace(0, 1, steps)              # normalized time
    # q = np.linspace(q_start, q_end, steps)    # shape (100,6)
    # from roboticstoolbox.backends.swift import Swift
    # robot.plot(q=q, block=True, loop=False, jointaxes=True, eeframe=True, shadow=True)
# q=[[1,1,1,1,1,1], [1.1,1.1,1.1,1.1,1.1,1.1]]
