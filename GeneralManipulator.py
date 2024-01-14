# Author: ivantregear
# Date Created: 07/11/2022
# Description: WiP

# When I wrote this code only got I and knew how it worked.
# Now only god knows it.

import matplotlib.pyplot as plt
import numpy as np
from Robot import SerialManipulator
from FrameTransformation import init_axis


def main():
    ax = init_axis()

    robot_type = 'SCARA'

    # DH Order = a, alpha, d, theta

    if robot_type == 'UR10e':
        def dh(inp):
            d_h = np.array([[0, np.pi / 2, 0.1807, inp[0]],
                           [-0.6127, 0, 0, inp[1]],
                           [-0.57155, 0, 0, inp[2]],
                           [0, np.pi / 2, 0.17415, inp[3]],
                           [0, -np.pi / 2, 0.11985, inp[4]],
                           [0, 0, 0.11655, inp[5]]], dtype='float')
            return d_h

        manipulator = SerialManipulator(dh, 6, ax)
        manipulator.show_coordinates = False

    elif robot_type == 'UR5e':
        def dh(inp):
            d_h = np.array([[0, np.pi / 2, 0.1625, inp[0]],
                           [-0.425, 0, 0, inp[1]],
                           [-0.3922, 0, 0, inp[2]],
                           [0, np.pi / 2, 0.1333, inp[3]],
                           [0, -np.pi / 2, 0.0997, inp[4]],
                           [0, 0, 0.0996, inp[5]]], dtype='float')
            return d_h

        manipulator = SerialManipulator(dh, 6, ax)
        manipulator.show_coordinates = False

    elif robot_type == 'SCARA':
        def dh(inp):
            d_h = np.array([[0, 0, 1, inp[0]],
                            [0, np.pi/2, 0, inp[1]],
                            [1, 0, 0, inp[2]],
                            [1, 0, 0, 0]], dtype='float')
            return d_h

        manipulator = SerialManipulator(dh, 3, ax)
        manipulator.show_coordinates = True

    n = 1
    # inputs = np.radians(np.array([[0, 0, 1]]))  # Joint angles, but could be R or P
    inputs = np.array([np.linspace(0, 2*np.pi, n),
                       np.zeros(n),
                       np.linspace(0, 0.7, n)]).T

    # Must be called in while True loop
    while True:
        for inp in inputs:
            manipulator.inputs = inp
            manipulator.update_position()
            manipulator.plot_position()
    # Must be called in while True loop
    plt.show()


if __name__ == "__main__":
    main()
