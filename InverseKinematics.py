from ForwardKinematics import robot
import numpy as np

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
    solutions, errors = compare_inverse_kinematic_solutions(robot, target_pose, q0=[0,0,0,0,0,0])

    for name, sol in solutions.items():
        print(f"{name} solution: {sol.q.round(4) if sol.success else 'No solution found'} and {sol}")
    print("Errors:")
    for name, error in errors.items():
        print(f"{name} error: {error.round(5) if error is not None else 'No solution found'}")