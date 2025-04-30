from ForwardKinematics import robot
import sympy as sp
import numpy as np

def compare_solutions(actual, me, tol=1e-5):
    """Compare the actual and symbolic solutions for the end-effector pose."""
    delta = actual - me

    delta_num = sp.lambdify(
        (theta1, theta2, theta3, theta4, theta5, theta6),
        delta,
        modules="numpy"
    )

    for i in range(100):
        # generate six random angles in [0, 2π)
        th = np.random.rand(6) * np.pi * 2

        # fast numeric eval
        diff = np.array(delta_num(*th), dtype=float)

        # compare in O(16) time
        if not np.allclose(diff, 0, atol=tol):
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
# T0e = sp.Matrix(robot.fkine([theta1, theta2, theta3, theta4, theta5, theta6]).A)
# z0e_actual = (T0e[0:3,2])

# z0e = sp.Matrix([
#     [(-c(theta1)*s(theta2 + theta3)*c(theta4) + s(theta1)*s(theta4))*s(theta5) + c(theta1)*c(theta2 + theta3)*c(theta5)],
#     [(-s(theta1)*s(theta2 + theta3)*c(theta4) - c(theta1)*s(theta4))*s(theta5) + s(theta1)*c(theta2 + theta3)*c(theta5)],
#     [ c(theta2 + theta3)*c(theta4)*s(theta5) + s(theta2 + theta3)*c(theta5)]
# ])

position_03 = T03[0:3, 3]

given_T0e = robot.fkine([.1,.1,.1,.1,.1,.1])
print(given_T0e)
e1 = sp.Eq(position_03[0], given_T0e.A[0, 3]-robot.d[5]*given_T0e.A[0, 2])
e2 = sp.Eq(position_03[1], given_T0e.A[1, 3]-robot.d[5]*given_T0e.A[1, 2])
e3 = sp.Eq(position_03[2], given_T0e.A[2, 3]-robot.d[5]*given_T0e.A[2, 2])

sp.pprint(e1)
sp.pprint(e2)
sp.pprint(e3)

eqs = [e1, e2, e3]
sol = sp.solve(eqs, (theta1, theta2, theta3), dict=True)

print(sol)