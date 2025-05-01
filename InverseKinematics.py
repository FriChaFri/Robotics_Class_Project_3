from ForwardKinematics import robot
import numpy as np
from spatialmath import SE3

def intersecting_circles(c1, r1, c2, r2):
    """
    Find the intersection points of two circles in 2D space.
    c1, c2: centers of the circles (x, y)
    r1, r2: radii of the circles
    Returns two tuple intersection points.
    """
    x1, y1 = c1
    x2, y2 = c2

    d = np.hypot(x2-x1, y2-y1) # distance between centers, hyplot more stable than sqrt

    a=(r1**2-r2**2+d**2)/(2*d)
    # b=(r2**2-r1**2+d**2)/(2*d) # redundant calculation since a is used. 

    h_sq = r1*r1 - a*a
    h_sq = max(h_sq, 0.0) # ensure non-negative for sqrt
    h = np.sqrt(h_sq)

    x5=x1+a*(x2-x1)/d
    y5=y1+a*(y2-y1)/d

    intersection_point_1 = [x5+h*(y2-y1)/d, y5-h*(x2-x1)/d]
    if h < 1e-8:
        intersection_point_2 = intersection_point_1
    else:
        intersection_point_2 = [x5-h*(y2-y1)/d, y5+h*(x2-x1)/d]

    # sort so the higher-z point (elbow-up) is first
    if intersection_point_1[1] >= intersection_point_2[1]:
        return intersection_point_1, intersection_point_2
    else:
        return intersection_point_2, intersection_point_1

def rz2xyz(r, theta, z):
    """
    Convert (r, z) coordinates to (x, y, z) coordinates. 
    Expects theta in radians.
    """
    x = r * np.cos(theta)
    y = r * np.sin(theta)
    return np.array([x, y, z])

def xyz2rz(wrist_center_xyz):
    """
    Convert [x, y, z] coordinates to [r, z] coordinates. 
    """
    x, y, z = wrist_center_xyz
    r = np.sqrt(x**2 + y**2)
    return np.array([r, z])



def closed_form_inverse_kinematics(robot, target_pose):
    """
    Closed form inverse kinematics solution for the robot.
    Input: robot object and target pose of the end effector in the base frame.
    """
    # First, find theta1
    theta1_pair = find_theta1(robot, target_pose)

    # Second, find theta2 and theta3 pairs
    theta2_theta3_pairs = find_theta2_theta3_pairs(robot, target_pose)

    # Third, combine theta1, theta2, and theta3 to form a list of all possible solutions for theta1, theta2, and theta3
    theta1_theta2_theta3_solutions = np.zeros((8, 3))
    for i in range(2):
        for j in range(4):
            theta1_theta2_theta3_solutions[i*4+j] = [theta1_pair[i], theta2_theta3_pairs[j][0], theta2_theta3_pairs[j][1]]

    return theta1_theta2_theta3_solutions

def find_wrist_center_xyz(robot, target_pose):
    """
    Find the wrist center of the robot in the base frame given the target pose of the end effector.
    Using formula: wrist center = target position - d6 * z rotation of target pose
    Return
    """
    # First, extract position and z rotation form target pose
    target_position = target_pose.t
    target_rotation = target_pose.R
    target_z_rotation = target_rotation[:,2]

    # Second, find wrist center xyz position
    wrist_position_xyz = target_position - robot.d[5]*target_z_rotation

    return wrist_position_xyz
    
def find_theta1(robot, target_pose):
    """
    Find theta1 for the closed for inverse kinematics solution. 
    Output is a list of theta 1 values.
    """
    # First, find the position of the wrist center
    xc, yc, _ = find_wrist_center_xyz(robot, target_pose)
    
    theta1_a = np.arctan2(yc, xc)
    theta1_b = np.pi + np.arctan2(yc, xc)

    return np.array([theta1_a, theta1_b])


def find_theta2_theta3_pairs(robot, target_pose):
    """
    Find theta2 and theta3 pairs for the closed form inverse kinematics solution.
    Output is a list of pairs of theta2 and theta3.
    The first element of the pair is theta2 and the second element is theta3.
    """

    #----Find position of all origins from 1 to wrist center----#
    # First, find the position of the wrist center
    wrist_position_xyz = find_wrist_center_xyz(robot, target_pose)
    ## find wrist center rz position
    wrist_position_rz = xyz2rz(wrist_position_xyz)

    # Second, find the position of the first origin
    origin1_rz = np.array([robot.a[0], robot.d[0]])

    # Third, define the radius of the circles intersecting at origin2
    ## from the wrist center, the radius of the circle to origin2 is fixed from length 3 and length 4
    radius_from_wrist_to_2 = np.sqrt((robot.a[2])**2+(robot.d[3])**2)
    ## from origin1, the radius of the circle to origin2 is fixed from length 2
    radius_from_1_to_2 = robot.a[1]

    # Fourth, find the intersection coordinates (r, z) of the circles intersecting at origin2
    origin2_a_rz, origin2_b_rz = intersecting_circles(wrist_position_rz, radius_from_wrist_to_2, origin1_rz, radius_from_1_to_2)

    # Fifth, find the radii of the circles that intersect at origin3
    ## the circle from origin2 to origin3 is fixed from length 3
    radius_from_2_to_3 = robot.a[2]
    ## the circle from the wrist center to origin3 is fixed from length 4
    radius_from_wrist_to_3 = robot.d[3]

    # Sixth, find the intersection coordinates (r,z) of the circles intersecting at origin3
    # These circles are centered at origin2 and origin3 and have radii of radius_from_2_to_3 and radius_from_wrist_to_3 respectively
    # There are two possible locations for origin2, so both will need to be evaluated, which will result in four possible locations for origin3
    origin3_a_rz, origin3_b_rz = intersecting_circles(origin2_a_rz, radius_from_2_to_3, wrist_position_rz, radius_from_wrist_to_3)
    origin3_c_rz, origin3_d_rz = intersecting_circles(origin2_b_rz, radius_from_2_to_3, wrist_position_rz, radius_from_wrist_to_3)

    # Seventh, find the possible angles theta2 and theta3
    # theta_i-1 = arctan2(z_i - z_i-1, r_i - r_i-1) for i = 2,3
    theta2_a = np.arctan2(origin2_a_rz[1] - origin1_rz[1], origin2_a_rz[0] - origin1_rz[0])             # angle between points 3a and 2
    theta2_b = np.arctan2(origin2_b_rz[1] - origin1_rz[1], origin2_b_rz[0] - origin1_rz[0])             # angle between points 3b and 2
    theta3_a = np.arctan2(origin3_a_rz[1]-origin2_a_rz[1], origin3_a_rz[0]-origin2_a_rz[0]) - theta2_a  # angle between points 4a and 3a
    theta3_b = np.arctan2(origin3_b_rz[1]-origin2_a_rz[1], origin3_b_rz[0]-origin2_a_rz[0]) - theta2_a  # angle between points 4b and 3a
    theta3_c = np.arctan2(origin3_c_rz[1]-origin2_b_rz[1], origin3_c_rz[0]-origin2_b_rz[0]) - theta2_b  # angle between points 4c and 3b
    theta3_d = np.arctan2(origin3_d_rz[1]-origin2_b_rz[1], origin3_d_rz[0]-origin2_b_rz[0]) - theta2_b  # angle between points 4d and 3b

    theta2_theta3_pairs = np.array([
                                     [theta2_a, theta3_a],
                                     [theta2_a, theta3_b],
                                     [theta2_b, theta3_c],
                                     [theta2_b, theta3_d]
                                                           ])
    return theta2_theta3_pairs

# def find_theta4_theta5_theta6(robot, target_pose, theta1, theta2, theta3):

def compare_inverse_kinematic_solutions(robot, target_pose, q0=None):
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

def check_inverse_kinematics_solution(robot, n,
                                     angle_range=(0, np.pi/4),
                                     tol=1e-6,
                                     verbose=True):
    """
    Test n random IK problems on robot in [min_ang, max_ang].
    If verbose=True, prints full ✅/❌ banners with pose, angles, solutions.
    Always prints a final summary including the range.
    """
    min_ang, max_ang = angle_range
    if max_ang <= min_ang:
        raise ValueError("angle_range max must be > min")

    total_correct = 0

    for i in range(n):
        # 1) sample in [min_ang, max_ang]
        expected = np.random.rand(6) * (max_ang - min_ang) + min_ang
        target_pose = robot.fkine(expected)

        # 2) get all closed-form solutions
        solutions = closed_form_inverse_kinematics(robot, target_pose)

        # 3) check with proper wrap-around for first 3 joints
        solution_found = False
        for j, sol in enumerate(solutions):
            # compute difference and wrap it into [-π, +π]
            raw_diff     = expected[:3] - sol[:3]
            wrapped_diff = (raw_diff + np.pi) % (2*np.pi) - np.pi

            if np.linalg.norm(wrapped_diff) < tol:
                solution_found = True
                total_correct += 1
                if verbose:
                    print("✅✅✅✅✅✅✅✅✅✅✅")
                    print(f"Trial #{i+1}, Solution #{j+1} matched within tol={tol}")
                    print(f"Target Pose:\n{target_pose}")
                    print(f"Expected Angles:\n{expected}  # [θ1, θ2, θ3, θ4, θ5, θ6]")
                    print(f"All Solutions:\n{solutions}  # [θ1, θ2, θ3]")
                    print(f"Matched diff (first 3 joints): {wrapped_diff}")
                    print("✅✅✅✅✅✅✅✅✅✅✅\n")
                break

        if not solution_found and verbose:
            print("❌❌❌❌❌❌❌❌❌❌❌❌")
            print(f"Trial #{i+1}: No solution found within tol={tol}")
            print(f"Target Pose:\n{target_pose}")
            print(f"Expected Angles:\n{expected}  # [θ1, θ2, θ3, θ4, θ5, θ6]")
            print(f"All Solutions:\n{solutions}  # [θ1, θ2, θ3]")
            print("❌❌❌❌❌❌❌❌❌❌❌❌\n")

    # final summary
    print(f"\n-- Summary for range {angle_range} rad --")
    print(f"Total correct ✅ solutions: {total_correct} out of {n}")
    pct = 100 * total_correct / n
    print(f"Success rate: {pct:.1f}%\n")



    
if __name__ == "__main__":
    # print("Target Pose:")
    # target_pose = robot.fkine([.1,.1,.1,.1,.1,.1])
    # print(target_pose)
    # solutions, errors = compare_inverse_kinematic_solutions(robot, target_pose, q0=[0,0,0,0,0,0])

    # for name, sol in solutions.items():
    #     print(f"{name} solution: {sol.q.round(4) if sol.success else 'No solution found'} and {sol}")
    # print("Errors:")
    # for name, error in errors.items():
    #     print(f"{name} error: {error.round(5) if error is not None else 'No solution found'}")

    
    #TODO define origin0,1,2,3,4,5,6 and everything else on a figure, including length 1, 2, and so on
    #TODO show robot on r-z plane


    given_pose = np.array([[.7551, .4013, .5184, 399.1255], 
                           [.6084, -.7235, -.3262, 171.01526],
                           [.2441, .5617, -.7905, 416.0308], 
                           [0, 0, 0, 1]])
    target_pose = SE3(given_pose, check=False)
    print(target_pose)
    solutions = closed_form_inverse_kinematics(robot, target_pose)
    print(np.round(solutions,3))

    solutions, errors = compare_inverse_kinematic_solutions(robot, target_pose)
    for name, sol in solutions.items():
        print(f"{name} solution: {sol.q.round(4) if sol.success else 'No solution found'} and {sol}")

    ''' #CODE FOR TESTING
    # number of IK tests per window
    n_tests = 100
    # fraction of π each window spans, e.g. 0.25 → π/4
    frac_step = 1/16
    # how many windows to run; e.g. 4 will cover [0,π] in π/4 slices
    num_steps = 32

    for i in range(num_steps):
        lo = i * frac_step * np.pi
        hi = (i + 1) * frac_step * np.pi
        print(f"\n=== Testing angle range [{lo:.2f}, {hi:.2f}] rad ===")
        check_inverse_kinematics_solution(
            robot,
            n_tests,
            angle_range=(lo, hi),
            verbose=False
        )

        check_inverse_kinematics_solution(
        robot,
        10,
        angle_range=(np.pi/2, np.pi),
        verbose=True,
        tol=0.1
    )
    '''

