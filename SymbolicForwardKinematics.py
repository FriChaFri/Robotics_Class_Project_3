from ForwardKinematics import robot
import sympy as sp
import random as rd
import numpy as np

def compare_solutions(T03_actual, T03_me, tol=1e-5):
    """Compare the actual and symbolic solutions for the end-effector pose."""
    delta = T03_actual - T03_me

    delta_num = sp.lambdify(
        (theta1, theta2, theta3, theta4, theta5, theta6),
        delta,
        modules="numpy"
    )

    for i in range(1000):
        # generate six random angles in [0, π)
        th = np.random.rand(6) * np.pi

        # fast numeric eval
        diff = np.array(delta_num(*th), dtype=float)

        # compare in O(16) time
        if not np.allclose(diff, np.zeros((4,4)), atol=tol):
            print("Mismatch at", th)
            break
        else:
            print("Passed")

s=sp.sin
c=sp.cos

theta1, theta2, theta3, theta4, theta5, theta6 = sp.symbols('theta1 theta2 theta3 theta4 theta5 theta6')

T03 = sp.Matrix([[-s(theta2+theta3)*c(theta1),  s(theta1), c(theta1)*c(theta2+theta3), -250*s(theta2)*c(theta1)-70*s(theta2+theta3)*c(theta1)+27.5*c(theta1)], 
                 [-s(theta1)*s(theta2+theta3), -c(theta1), s(theta1)*c(theta2+theta3), -250*s(theta1)*s(theta2)-70*s(theta1)*s(theta2+theta3)+27.5*s(theta1)], 
                 [ c(theta2+theta3),                0,            s(theta2+theta3),                  250*c(theta2)+70*c(theta2+theta3)+339                  ],
                 [     0,                           0,                   0,                                           1                                     ]])

# T_all = robot.fkine_all([theta1, theta2, theta3, theta4, theta5, theta6])
# T03_actual = sp.Matrix(T_all[3].A)

