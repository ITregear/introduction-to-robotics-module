# Author: ivantregear
# Date Created: 09/01/2023
# Description: 

# When I wrote this code only got I and knew how it worked.
# Now only god knows it.

import sympy as smp


def main():

    smp.init_printing(use_unicode=True, wrap_line=False)

    t1, t2, t3 = smp.symbols("theta1 theta2 theta3")
    x, y = smp.symbols("x y")
    l1, l2 = smp.symbols("l1 l2")

    b = smp.symbols("beta")

    s = smp.sin
    c = smp.cos

    m1 = smp.Matrix([[c(t1+t2+t3), -s(t1+t2+t3), 0, l1*c(t1) + l2*c(t1+t2)],
                     [s(t1+t2+t3), c(t1+t2+t3), 0, l1*s(t1)+l2*s(t1+t2)],
                     [0, 0, 1, 0],
                     [0, 0, 0, 1]])

    m2 = smp.Matrix([[c(b), -s(b), 0, x],
                     [s(b), c(b), 0, y],
                     [0, 0, 1, 0],
                     [0, 0, 0, 1]])

    e1 = smp.Eq()

    # result = smp.solve([e1, e2, e3, e4], (t1, t2))

    result = smp.solve([m1, m2], *(r1, r2, r3, r4))

    print(result)


if __name__ == "__main__":
    main()
