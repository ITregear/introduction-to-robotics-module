# Author: ivantregear
# Date Created: 09/01/2023
# Description: 

# When I wrote this code only got I and knew how it worked.
# Now only god knows it.

import sympy as smp


def main():
    smp.init_printing(use_unicode=True, wrap_line=False)

    l1, l2, l3, l4, l5 = smp.symbols("L1 L2 L3 L4 L5")
    t1, t2, d3, t4 = smp.symbols("theta1 theta2 d3 theta4")

    t1_d, t2_d, d3_d, t4_d = smp.symbols("thetadot_1 thetadot_2 ddot_3 thetadot_4")

    s = smp.sin
    c = smp.cos

    s1 = smp.Function('s')
    c1 = smp.Function('c')

    # Transformation Matrices
    t_0_1 = smp.Matrix([[c(t1), -s(t1), 0, 0],
                        [s(t1), c(t1), 0, 0],
                        [0, 0, 1, l1],
                        [0, 0, 0, 1]])

    t_1_2 = smp.Matrix([[c(t2), -s(t2), 0, l2],
                        [0, 0, -1, -l3],
                        [s(t2), c(t2), 0, 0],
                        [0, 0, 0, 1]])

    t_2_3 = smp.Matrix([[1, 0, 0, 0],
                        [0, 0, -1, d3],
                        [0, 1, 0, 0],
                        [0, 0, 0, 1]])

    t_3_4 = smp.Matrix([[c(t4), -s(t4), 0, 0],
                        [s(t4), c(t4), 0, 0],
                        [0, 0, 1, 0],
                        [0, 0, 0, 1]])

    t_4_e = smp.Matrix([[0, -1, 0, l4],
                        [1, 0, 0, 0],
                        [0, 0, 1, l5],
                        [0, 0, 0, 1]])

    # Base (fixed)

    omega_0_0 = smp.Matrix([[0, 0, 0]]).T
    v_0_0 = smp.Matrix([[0, 0, 0]]).T

    # 1st joint (revolute)

    omega_1_1 = t_0_1[:-1, :-1].T * omega_0_0 + smp.Matrix([[0, 0, t1_d]]).T
    v_1_1 = t_0_1[:-1, :-1].T * (v_0_0 + omega_0_0.cross(smp.Matrix([[0, 0, l1]])))

    # 2nd joint (revolute)

    omega_2_2 = t_1_2[:-1, :-1].T * omega_1_1 + smp.Matrix([[0, 0, t2_d]]).T
    v_2_2 = t_1_2[:-1, :-1].T * (v_1_1 + omega_1_1.cross(smp.Matrix([[l2, -l3, 0]]).T))

    # 3rd joint (prismatic)

    omega_3_3 = t_2_3[:-1, :-1].T * omega_2_2
    v_3_3 = t_2_3[:-1, :-1].T * (v_2_2 + omega_2_2.cross(smp.Matrix([[0, -d3, 0]]).T)) + smp.Matrix([[0, 0, d3_d]]).T

    # 4th joint (revolute)

    omega_4_4 = t_3_4[:-1, :-1].T * omega_3_3 + smp.Matrix([[0, 0, t4_d]]).T
    v_4_4 = t_3_4[:-1, :-1].T * (v_3_3 + omega_3_3.cross(smp.Matrix([[0, 0, 0]]).T))

    # End effector (fixed)

    omega_e_e = t_4_e[:-1, :-1].T * omega_4_4
    v_e_e = t_4_e[:-1, :-1].T * (v_4_4 + omega_4_4.cross(smp.Matrix([[l4, 0, l5]]).T))

    # Jacobian

    t_0_e = t_0_1*t_1_2*t_2_3*t_3_4*t_4_e

    jac_rot = smp.Matrix([[t_0_e[:-1, :-1], smp.zeros(3, 3)],
                          [smp.zeros(3, 3), t_0_e[:-1, :-1]]])

    j_e = smp.linear_eq_to_matrix(smp.Matrix([[v_e_e],
                                              [omega_e_e]]), *[t1_d, t2_d, d3_d, t4_d])

    j_0 = jac_rot * j_e[0]

    smp.pprint(smp.trigsimp(j_0).replace(s, s1).replace(c, c1), use_unicode=True)


if __name__ == "__main__":
    main()
