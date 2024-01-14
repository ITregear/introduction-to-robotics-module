# Author: ivantregear
# Date Created: 07/01/2023
# Description: 

# When I wrote this code only got I and knew how it worked.
# Now only god knows it.

import numpy as np


def main():
    np.set_printoptions(formatter={'float': lambda x: "{0:0.3f}".format(x)})

    l1 = 0.4
    l2 = -0.2
    l3 = 0.1
    l4 = 0.1
    l5 = 0.25

    inputs = [0, 0, 0, 0]

    T_0_1 = np.array([[np.cos(inputs[0]), -np.sin(inputs[0]), 0, 0],
                      [np.sin(inputs[0]), np.cos(inputs[0]), 0, 0],
                      [0, 0, 1, l1],
                      [0, 0, 0, 1]])

    T_0_2 = np.array([[np.cos(inputs[0]) * np.cos(inputs[1]), -np.cos(inputs[0]) * np.sin(inputs[1]), np.sin(inputs[0]),
                       l2 * np.cos(inputs[0]) - l3 * np.sin(inputs[0])],
                      [np.sin(inputs[0]) * np.cos(inputs[1]), -np.sin(inputs[0]) * np.sin(inputs[1]), -np.cos(inputs[0]),
                       l2 * np.sin(inputs[0]) - l3 * np.cos(inputs[0])],
                      [np.sin(inputs[1]), np.cos(inputs[1]), 0, l1],
                      [0, 0, 0, 1]
                      ])

    T_0_3 = np.array([[np.cos(inputs[0]) * np.cos(inputs[1]), np.sin(inputs[0]), np.cos(inputs[0]) * np.sin(inputs[1]),
                       -inputs[2] * np.cos(inputs[0]) * np.sin(inputs[1]) + l2 * np.cos(inputs[0]) - l3 * np.sin(
                           inputs[0])],
                      [np.sin(inputs[0]) * np.cos(inputs[1]), -np.cos(inputs[0]), np.sin(inputs[0]) * np.sin(inputs[1]),
                       -inputs[2] * np.sin(inputs[0]) * np.sin(inputs[1]) + l2 * np.sin(inputs[0]) - l3 * np.cos(
                           inputs[0])],
                      [np.sin(inputs[1]), 0, -np.cos(inputs[1]), inputs[2]*np.cos(inputs[1]) + l1],
                      [0, 0, 0, 1]
                      ])

    T_0_4 = np.array([[np.cos(inputs[0]) * np.cos(inputs[0]) * np.cos(inputs[3]) + np.sin(inputs[0]) * np.sin(
                          inputs[3]),
                       -np.cos(inputs[0]) * np.cos(inputs[1]) * np.sin(inputs[3]) + np.sin(inputs[0]) * np.cos(
                           inputs[3]),
                       np.cos(inputs[0]) * np.sin(inputs[1]),
                       -inputs[2] * np.cos(inputs[0]) * np.sin(inputs[1]) + l2 * np.cos(inputs[0]) - l3 * np.sin(
                           inputs[0])],
                      [np.sin(inputs[0]) * np.cos(inputs[0]) * np.cos(inputs[3]) - np.cos(inputs[0]) * np.sin(
                          inputs[3]),
                       -np.sin(inputs[0]) * np.cos(inputs[1]) * np.sin(inputs[3]) - np.cos(inputs[0]) * np.cos(
                           inputs[3]),
                       np.sin(inputs[0]) * np.sin(inputs[1]),
                       -inputs[2] * np.sin(inputs[0]) * np.sin(inputs[1]) + l2 * np.sin(inputs[0]) - l3 * np.cos(
                           inputs[0])],
                      [np.sin(inputs[1])*np.cos(inputs[3]), -np.sin(inputs[1])*np.sin(inputs[3]), -np.cos(inputs[1]),
                       l1 + inputs[2] * np.cos(inputs[1])],
                      [0, 0, 0, 1]
                      ], dtype='float')

    T_0_e = np.array([[-np.cos(inputs[0]) * np.cos(inputs[1]) * np.sin(inputs[3]) + np.sin(inputs[0]) * np.cos(
        inputs[3]),
                       -np.cos(inputs[0]) * np.cos(inputs[1]) * np.cos(inputs[3]) - np.sin(inputs[0]) * np.sin(
                           inputs[3]),
                       np.cos(inputs[0]) * np.sin(inputs[1]),
                       l4 * (np.cos(inputs[0]) * np.cos(inputs[1]) * np.cos(inputs[3]) + np.sin(inputs[0]) * np.sin(
                           inputs[3]))
                       + l5 * np.cos(inputs[0]) * np.sin(inputs[1]) - inputs[2] * np.cos(inputs[0]) * np.sin(inputs[1])
                       + l2 * np.cos(inputs[0]) - l3 * np.sin(inputs[0])],
                      [-np.sin(inputs[0]) * np.cos(inputs[1]) * np.sin(inputs[3]) - np.cos(inputs[0]) * np.cos(
                          inputs[3]),
                       -np.sin(inputs[0]) * np.cos(inputs[1]) * np.cos(inputs[3]) + np.cos(inputs[0]) * np.sin(
                           inputs[3]),
                       np.sin(inputs[0]) * np.sin(inputs[1]),
                       l4 * (np.sin(inputs[0]) * np.cos(inputs[1]) * np.cos(inputs[3]) - np.cos(inputs[0]) * np.sin(
                           inputs[3]))
                       + l5 * np.sin(inputs[0]) * np.sin(inputs[1]) - inputs[2] * np.sin(inputs[0]) * np.sin(inputs[1])
                       + l2 * np.sin(inputs[0]) - l3 * np.cos(inputs[0])],
                      [-np.sin(inputs[1]) * np.sin(inputs[3]), -np.sin(inputs[1]) * np.cos(inputs[3]),
                       -np.cos(inputs[1]),
                       l4 * np.sin(inputs[1]) * np.cos(inputs[3]) - l5 * np.cos(inputs[1]) + inputs[2] * np.cos(
                           inputs[1]) + l1],
                      [0, 0, 0, 1]], dtype='float')

    print("O_1T:")
    print(T_0_1)
    print()
    print("O_2T:")
    print(T_0_2)
    print()
    print("O_3T:")
    print(T_0_3)
    print()
    print("O_4T:")
    print(T_0_4)
    print()
    print("O_5T:")
    print(T_0_e)


if __name__ == "__main__":
    main()
