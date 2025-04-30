from ForwardKinematics import robot
import numpy as np

def intersecting_circles(c1, r1, c2, r2):
    """
    Find the intersection points of two circles in 2D space.
    c1, c2: centers of the circles (x, y)
    r1, r2: radii of the circles
    Returns two tuple intersection points.
    """
    x1, y1 = c1
    x2, y2 = c2

    d=np.sqrt((x2-x1)**2 + (y2-y1)**2)

    a=(r1**2-r2**2+d**2)/(2*d)
    # b=(r2**2-r1**2+d**2)/(2*d) # redundant calculation since a is used. 

    h=np.sqrt(r1**2-a**2)

    x5=x1+a*(x2-x1)/d
    y5=y1+a*(y2-y1)/d

    intersection_point_1 = [x5+h*(y2-y1)/d, y5-h*(x2-x1)/d]
    intersection_point_2 = [x5-h*(y2-y1)/d, y5+h*(x2-x1)/d]

    return intersection_point_1, intersection_point_2

def rz2xyz(r, theta, z):
    """
    Convert (r, z) coordinates to (x, y, z) coordinates. 
    Expects theta in radians.
    """
    x = r * np.cos(theta)
    y = r * np.sin(theta)
    return np.array([x, y, z])

def xyz2rz(x, y, z):
    """
    Convert (x, y, z) coordinates to (r, z) coordinates. 
    """
    r = np.sqrt(x**2 + y**2)
    return np.array([r, z])



def closed_form_inverse_kinematics(robot, target_pose):
    """
    """
    return None
    
def compare_inverse_kinematic_solutions(robot, target_pose, q0):
    """
    compare the closed form inverse kinematics solution with the numerical inverse kinematics solutions built into the robot class.
    """
    # run each solver
    sol_lm = robot.ikine_LM(target_pose, q0=q0) # Levemberg-Marquadt Numerical Inverse Kinematics Solver
    sol_nr = robot.ikine_NR(target_pose, q0=q0) # Newton-Raphson Numerical Inverse Kinematics Solver
    sol_gn = robot.ikine_GN(target_pose, q0=q0) # Gauss-Newton Numerical Inverse Kinematics Solver
    sol_cf = closed_form_inverse_kinematics(robot, target_pose)
    # print(sol_cf)
    solutions = {'LM': sol_lm, 'NR': sol_nr, 'GN': sol_gn}

    # compute end-effector pose error for each
    errors = {}
    for name, sol in solutions.items():
        if sol.success:
            T_sol = robot.fkine(sol.q)
            errors[name] = np.linalg.norm(T_sol.A - target_pose.A)
        else:
            errors[name] = None

    return solutions, errors


if __name__ == "__main__":
    print("Target Pose:")
    target_pose = robot.fkine([.1,.1,.1,.1,.1,.1])
    print(target_pose)
    # solutions, errors = compare_inverse_kinematic_solutions(robot, target_pose, q0=[0,0,0,0,0,0])

    # for name, sol in solutions.items():
    #     print(f"{name} solution: {sol.q.round(4) if sol.success else 'No solution found'} and {sol}")
    # print("Errors:")
    # for name, error in errors.items():
    #     print(f"{name} error: {error.round(5) if error is not None else 'No solution found'}")


    #----Find position of all origins from 1 to wrist center----#
    # First, find the position of the wrist center
    ## extract position and z rotation form target pose
    target_position = target_pose.t
    target_rotation = target_pose.R
    target_z_rotation = target_rotation[:,2]
    ## find wrist center xyz position
    wrist_position_xyz = target_position - robot.d[5]*target_z_rotation
    ## find wrist center rz position
    wrist_position_rz = xyz2rz(wrist_position_xyz[0], wrist_position_xyz[1], wrist_position_xyz[2])

    # Second, find the position of the first origin
    origin2_rz = np.array([robot.a[0], robot.d[0]])

    # Third, define the radius of the circles intersecting at origin3
    ## from the wrist center, the radius of the circle to origin3 is fixed from length 3 and length 4
    radius_from_wrist_to_3 = np.sqrt((robot.a[2])**2+(robot.d[3])**2)
    ## from origin2, the radius of the circle to origin3 is fixed from length 2
    radius_from_2_to_3 = robot.a[1]

    # Fourth, find the intersection coordinates (r, z) of the circles intersecting at origin3
    origin3_a_rz, origin3_b_rz = intersecting_circles(wrist_position_rz, radius_from_wrist_to_3, origin2_rz, radius_from_2_to_3)
    print(origin3_a_rz, origin3_b_rz)

    # Fifth, find the radii of the circles that intersect at origin4
    ## the circle from origin3 to origin4 is fixed from length 3
    radius_from_3_to_4 = robot.a[2]
    ## the circle from the wrist center to origin4 is fixed from length 4
    radius_from_wrist_to_4 = robot.d[3]

    # Sixth, find the intersection coordinates (r,z) of the circles intersecting at origin4
    # These circles are centered at origin3 and origin4 and have radii of radius_from_3_to_4 and radius_from_wrist_to_4 respectively
    # There are two possible locations for origin3, so both will need to be evaluated, which will result in four possible locations for origin4
    origin4_a_rz, origin4_b_rz = intersecting_circles(origin3_a_rz, radius_from_3_to_4, wrist_position_rz, radius_from_wrist_to_4)
    origin4_c_rz, origin4_d_rz = intersecting_circles(origin3_b_rz, radius_from_3_to_4, wrist_position_rz, radius_from_wrist_to_4)

    # Seventh, find the possible angles theta2 and theta3
    # FIXME *Note: these angles are found assuming theta1 only has one solution, but theta1 has two solutions, so theta2 and theta3 will need to be evaluated for both theta1 solutions
    # theta_i = arctan2(z_i - z_i-1, r_i - r_i-1) for i = 2,3
    theta2_a = np.arctan2(origin3_a_rz[1] - origin2_rz[1], origin3_a_rz[0] - origin2_rz[0]) # angle between points 3a and 2
    theta2_b = np.arctan2(origin3_b_rz[1] - origin2_rz[1], origin3_b_rz[0] - origin2_rz[0]) # angle between points 3b and 2
    theta3_a = np.arctan2(origin4_a_rz[1]-origin3_a_rz[1], origin4_a_rz[0]-origin3_a_rz[0]) # angle between points 4a and 3a
    theta3_b = np.arctan2(origin4_b_rz[1]-origin3_b_rz[1], origin4_b_rz[0]-origin3_b_rz[0]) # angle between points 4b and 3b
    theta3_c = np.arctan2(origin4_c_rz[1]-origin3_a_rz[1], origin4_c_rz[0]-origin3_a_rz[0]) # angle between points 4c and 3a
    theta3_d = np.arctan2(origin4_d_rz[1]-origin3_b_rz[1], origin4_d_rz[0]-origin3_b_rz[0]) # angle between points 4d and 3b

    #TODO find theta 1
    #TODO define origin0,1,2,3,4,5,6 and everything else on a figure, including length 1, 2, and so on
    #TODO show robot on r-z plane

    # circle1 = 