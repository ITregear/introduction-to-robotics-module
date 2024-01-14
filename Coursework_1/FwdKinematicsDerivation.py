# Author: ivantregear
# Date Created: 08/01/2023
# Description: 

# When I wrote this code only got I and knew how it worked.
# Now only god knows it.

import sympy as smp


def main():
    smp.init_printing(use_unicode=True, wrap_line=False)

    l1, l2, l3, l4, l5 = smp.symbols("L1 L2 L3 L4 L5")
    t1, t2, d3, t4 = smp.symbols("theta1 theta2 d3 theta4")

    t_0_1 = smp.Matrix([[smp.cos(t1), -smp.sin(t1), 0, 0],
                        [smp.sin(t1), smp.cos(t1), 0, 0],
                        [0, 0, 1, l1],
                        [0, 0, 0, 1]])

    t_1_2 = smp.Matrix([[smp.cos(t2), -smp.sin(t2), 0, l2],
                        [0, 0, -1, -l3],
                        [smp.sin(t2), smp.cos(t2), 0, 0],
                        [0, 0, 0, 1]])

    t_2_3 = smp.Matrix([[1, 0, 0, 0],
                        [0, 0, -1, d3],
                        [0, 1, 0, 0],
                        [0, 0, 0, 1]])

    t_3_4 = smp.Matrix([[smp.cos(t4), -smp.sin(t4), 0, 0],
                        [smp.sin(t4), smp.cos(t4), 0, 0],
                        [0, 0, 1, 0],
                        [0, 0, 0, 1]])

    t_4_e = smp.Matrix([[0, -1, 0, l4],
                        [1, 0, 0, 0],
                        [0, 0, 1, l5],
                        [0, 0, 0, 1]])

    t_0_e = t_0_1*t_1_2*t_2_3*t_3_4*t_4_e

    inputs = [0, 0.3, 0, 0]

    smp.pprint(t_0_e.subs({t1:inputs[0], t2:inputs[1], d3:inputs[2], t4:inputs[3],
                           l1:0.4, l2:-0.2, l3:0.1, l4:0.1, l5:0.25}).evalf())
    smp.pprint(smp.trigsimp(t_0_e))


if __name__ == "__main__":
    main()
