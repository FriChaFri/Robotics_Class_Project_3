import numpy as np
from spatialmath import SE3
from roboticstoolbox import DHRobot, RevoluteDH


links = [ # in millimeters and radians
    RevoluteDH(alpha=-np.pi/2, a=-27.5,d=339,  offset=np.pi    ),  # link 1
    RevoluteDH(alpha=0,        a=250 , d=  0,  offset=-np.pi/2 ),  # link 2
    RevoluteDH(alpha=-np.pi/2, a=-70 , d=  0,  offset=np.pi    ),  # link 3
    RevoluteDH(alpha=np.pi/2,  a=  0 , d=250,  offset=0        ),  # link 4
    RevoluteDH(alpha=-np.pi/2, a=  0 , d=  0,  offset=0        ),  # link 5
    RevoluteDH(alpha=0,        a=  0 , d= 95,  offset=np.pi    ),  # link 6
]

robot = DHRobot(links, name="miniBOT6-R")


def forward_kinematics(theta_list, prismatic_ds=None):
    """
    theta_list : list of 6 θ values (for revolute joints)
    prismatic_ds: optional list of 6 d values (for prismatic joints)
                  only used if you have any PrismaticDH links.
    """
    # If you have prismatic joints, pass their ds in via `q` too:
    if prismatic_ds:
        q = [d if isinstance(link, PrismaticDH) else th
             for link, (th, d) in zip(robot.links, zip(theta_list, prismatic_ds))]
    else:
        q = theta_list

    T = robot.fkine(q)    # returns an SE3 object
    return T


def extract_pose(T: SE3):
    """
    Given an SE3 transform, returns:
      - position:    (3,) array
      - orientation: Euler Z‑YX angles (rad)
    """
    pos = T.t
    rpy = T.rpy(order="zyx", unit="rad")
    return pos, rpy


# --- example usage ---
if __name__ == "__main__":
    # Suppose we want the home configuration:
    # thetas = [0, np.pi/2, 0, 0, -np.pi/2, 0]  # override any offsets as needed
    # T_ee = forward_kinematics(thetas)
    # pos, euler_zyx = extract_pose(T_ee)

    # print("End‑effector pose:")
    # print(T_ee)                  # pretty‑printed 4×4 matrix
    # print("Position:", pos)      # 3‑vector
    # print("Euler ZYX:", euler_zyx)  # roll, pitch, yaw

    robot.__str__()
