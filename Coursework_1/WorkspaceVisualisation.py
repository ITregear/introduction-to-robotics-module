# Author: ivantregear
# Date Created: 05/01/2023
# Description:

# When I wrote this code only got I and knew how it worked.
# Now only god knows it.


import numpy as np
import matplotlib.pyplot as plt
import sys
import scipy

sys.path.append("../")

from Robot import SerialManipulator


def init_axis():
    ax = plt.figure().add_subplot(projection='3d')
    ax.set_xlim([-1.5, 1.5])
    ax.set_ylim([-1.5, 1.5])
    ax.set_zlim([0, 2])
    ax.set_xlabel("X")
    # ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_proj_type('ortho')
    plt.tight_layout()

    return ax


def main():
    np.set_printoptions(formatter={'float': lambda x: "{0:0.3f}".format(x)})
    ax = init_axis()

    l1 = 0.4
    l2 = -0.2
    l3 = 0.1
    l4 = 0.1
    l5 = 0.25

    # a_i-1, alpha_i-1, d_i, theta_i
    def dh(inputs):
        d_h = np.array([[0, 0, l1, inputs[0]],
                        [np.pi/2, l2, l3, inputs[1]],
                        [np.pi/2, 0, inputs[2], 0],
                        [0, 0, 0, inputs[3]],
                        [0, l4, l5, np.pi/2]], dtype='float')

        return d_h

    rrpr = SerialManipulator(dh, 5, ax)
    rrpr.show_coordinates = True

    # Assumptions to simplify workspace calculation
    # Firstly there is no deadzone at inside (can be shown using CAD model), as robot is able to reach itself at all
    # Internal areas
    # Due to first joint being revolute about global z, the workspace is defined by the maximum stretch of the arm
    # And is therefore max when d3 is max
    # This means only two parameters need to be varied
    # The max values of all points can then be calculated for a 2D slice
    # This can then be revolved around the global z-axis to creat the 3D workspace

    n = 100
    d3_max = 1

    theta1_range = np.linspace(0, 2*np.pi, n)
    theta2_range = np.linspace(0, 2*np.pi, n)
    d3_range = np.linspace(0, 1, n)
    theta3_range = np.linspace(0, 2*np.pi, n)

    end_effector_positions = np.empty((1, 3))

    for theta2 in theta2_range:
        for theta3 in theta3_range:

            rrpr.inputs = [0, theta2, d3_max, theta3]
            rrpr.update_position()

            end_effector_positions = np.concatenate((end_effector_positions, [rrpr.end_effector_position]))

    end_effector_positions[:, 1] = 0
    end_effector_positions = np.delete(end_effector_positions, np.where(end_effector_positions[:, 2] < 0)[0], axis=0)
    end_effector_positions = np.delete(end_effector_positions, np.where(end_effector_positions[:, 0] > 0)[0], axis=0)

    workspace = np.empty((1, 3))

    ax.scatter(*zip(*end_effector_positions), c='r')
    end_effector_positions[:, 0] = -end_effector_positions[:, 0]
    ax.scatter(*zip(*end_effector_positions), c='r')
    ax.view_init(azim=90, elev=0)
    ax.yaxis.set_ticklabels([])

    for theta1 in theta1_range:
        mat_rz = np.array([[np.cos(theta1), -np.sin(theta1), 0],
                           [np.sin(theta1), np.cos(theta1), 0],
                           [0, 0, 1]])

        rotated_frame = np.matmul(mat_rz, end_effector_positions.T).T

        workspace = np.concatenate((workspace, rotated_frame))

    workspace = np.delete(workspace, 0, 0)

    # X, Y = np.meshgrid(np.linspace(-1.5, 1.5, len(workspace[:])), np.linspace(-1.5, 1.5, len(workspace[:])))
    # Z = scipy.interpolate.griddata((workspace[:, 0], workspace[:, 1]), workspace[:, 2], (X, Y), method='linear')

    rrpr.inputs = [0, 3*np.pi/4, 0.75, 0]
    rrpr.update_position()
    rrpr.plot_position()
    # ax.scatter(*zip(*workspace), c='green')
    # ax.plot_surface(*zip(*workspace), color='green', alpha=0.1)
    # ax.plot_surface(X, Y, Z, color='green', alpha=0.1)
    # ax.plot_wireframe(X, Y, Z, rstride=5, cstride=5, color='green', alpha=0.3)
    plt.show()


if __name__ == "__main__":
    main()
