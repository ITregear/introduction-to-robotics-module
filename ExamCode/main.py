# Author: ivantregear
# Date Created: 30/03/2023
# Description: 

# When I wrote this code only God I and knew how it worked.
# Now only god knows it.

import sympy as smp
from RobotDerivations import RobotDerivations


def python_print(robot):
    robot.print_forward_kinematics()
    print("\n\n------------------------------------------------------------------------\n\n")
    robot.print_velocities()
    robot.print_accelerations()
    print("\n\n------------------------------------------------------------------------\n\n")
    robot.print_jacobian()
    print("\n\n------------------------------------------------------------------------\n\n")
    robot.print_forces()


def derivations():

    """
    DH-Table should be in following order:
    a_(i-1), alpha_(i-1), d_i, theta_i

    There must be the same number of joints as rows in DH table.
    Joints are 0, 1 or 2
    0 == Prismatic
    1 == Revolute
    2 == Fixed

    """

    # <editor-fold desc="Symbol Declaration">
    l1, l2, l3, l4, l5, l6 = smp.symbols('L1, L2, L3, L4, L5, L6')
    t1, t2, t3, t4, t5, t6 = smp.symbols('theta1, theta2, theta3, theta4, theta5, theta6')
    d1, d2, d3, d4, d5, d6 = smp.symbols('d1, d2, d3, d4, d5, d6')
    a, b, le = smp.symbols("a, b, le")
    lg1, lg2, lg3 = smp.symbols('lg1, lg2, lg3')

    pi = smp.pi
    # </editor-fold>

    dh_table = smp.Matrix([[0, 0, 0, t1],
                           [0, pi/2, d2+l1, 0],
                           [0, -pi/2, 0, t3-pi/2],
                           [l3, 0, 0, 0]])

    robot = RobotDerivations(dh=dh_table, joints=[1, 0, 1, 2])
    robot.com_symbols = [lg1, lg2, lg3]  # Add positions of COM. Assumed in same direction as link length
    robot.forward_kinematics(print_working=False)

    robot.derive_jacobian()

    robot.outward_iteration(gravity=False, vertical='y')
    robot.inward_iteration(f_e=[0, 0, 0], m_e=[0, 0, 0])

    return robot


if __name__ == "__main__":
    robot = derivations()
    python_print(robot)
