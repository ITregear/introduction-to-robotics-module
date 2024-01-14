# Author: ivantregear
# Date Created: 05/01/2023
# Description: 

# When I wrote this code only got I and knew how it worked.
# Now only god knows it.


import numpy as np
import matplotlib.pyplot as plt
import sys
import keyboard
import time

sys.path.append("../")

from Robot import SerialManipulator
from ParabolicTrajectory import blended_parabolic_motion


def init_axis():
    ax = plt.figure().add_subplot(projection='3d')
    ax.set_xlim([-1, 1])
    ax.set_ylim([-1, 1])
    ax.set_zlim([0, 1.5])
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    # ax.yaxis.set_ticklabels([])
    ax.set_proj_type('ortho')
    ax.view_init(azim=0, elev=35.264)
    plt.tight_layout()

    return ax


def find_closest(array, value):
    return (np.abs(array - value)).argmin()


def main():
    np.set_printoptions(formatter={'float': lambda x: "{0:0.3f}".format(x)})
    ax = init_axis()

    l1 = 0.4
    l2 = -0.2
    l3 = 0.1
    l4 = 0.1
    l5 = 0.25

    # alpha_i-1, a_i-1, d_i, theta_i
    def dh(inputs):
        d_h = np.array([[0, 0, l1, inputs[0]],
                        [np.pi/2, l2, l3, inputs[1]],
                        [np.pi/2, 0, inputs[2], 0],
                        [0, 0, 0, inputs[3]],
                        [0, l4, l5, np.pi/2]], dtype='float')

        return d_h

    rrpr = SerialManipulator(dh, 5, ax)
    rrpr.show_coordinates = True

    inputs = np.array([[0, np.pi / 2, 0.75, np.pi / 2],
                       [np.pi / 2, np.pi / 2, 0.75, 3 * np.pi / 2],
                       [np.pi, np.pi / 2, 0.95, np.pi / 2],
                       [3 * np.pi / 2, np.pi / 2, 0.4, 0],
                       [3 * np.pi / 2, 3 * np.pi / 4, 0.75, 0],
                       [np.pi, 3 * np.pi / 4, 0.95, np.pi / 8],
                       [np.pi, 3 * np.pi / 4, 0.9, np.pi / 4],
                       [np.pi, 3 * np.pi / 4, 0.8, 3 * np.pi / 8],
                       [np.pi, 3 * np.pi / 4, 0.7, np.pi / 2],
                       [np.pi / 2, 7 * np.pi / 8, 0.7, np.pi / 2],
                       [0, np.pi, 0.7, 3 * np.pi / 4],
                       [0, np.pi, 0.75, np.pi]])

    accelerations = np.array([np.ones(len(inputs)) * 0.5,
                              np.ones(len(inputs)) * 5,
                              np.ones(len(inputs)) * 15,
                              np.ones(len(inputs)) * 5])

    times = np.array([3, 2, 5, 4, 5, 2, 1, 5, 4, 3, 3])

    t, pos = blended_parabolic_motion(inputs, accelerations, times, n=1000)

    pos[:, 1] += np.pi / 2
    pos[:, 2] += 0.75

    # Generating plot of via points
    points_cartesian_space = np.zeros([len(inputs), 3])
    interp_cartesian_space = []
    for i, inp in enumerate(inputs):

        rrpr.inputs = inp
        rrpr.update_position()
        points_cartesian_space[i, :] = rrpr.end_effector_position

    trajectory, = ax.plot([], [], [], c='C1')
    ax.scatter(*zip(*points_cartesian_space), c='C1', marker='x')

    i = 0
    start_flag = False
    t0 = 0
    current_idx = 0

    while True:

        if keyboard.is_pressed('f'):
            t0 = time.perf_counter()
            start_flag = True

        if start_flag:
            current_time = time.perf_counter() - t0
            current_idx = find_closest(t, current_time)

        rrpr.inputs = pos[current_idx]
        rrpr.update_position()
        rrpr.plot_position()
        interp_cartesian_space += [rrpr.end_effector_position]

        trajectory.set_data_3d(*zip(*interp_cartesian_space))
        ax.view_init(azim=(current_idx * 360 / len(pos)), elev=35.264)


if __name__ == "__main__":
    main()
