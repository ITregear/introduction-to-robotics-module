# Author: ivantregear
# Date Created: 06/11/2022
# Description: WiP

# When I wrote this code only got I and knew how it worked.
# Now only god knows it.


import numpy as np
import matplotlib.pyplot as plt
from FrameTransformation import CoordinateAxis
from FrameTransformation import init_axis


def link_transformation(dh, i):

    m_T = np.array([[np.cos(dh[i, 3]), -np.sin(dh[i, 3]), 0, dh[i-1, 0]],
                    [np.sin(dh[i, 3]) * np.cos(dh[i-1, 1]), np.cos(dh[i, 3]) * np.cos(dh[i-1, 1]), -np.sin(dh[i-1, 1]),
                     -np.sin(dh[i-1, 1]) * dh[i, 2]],
                    [np.sin(dh[i, 3]) * np.sin(dh[i-1, 1]), np.cos(dh[i, 3]) * np.sin(dh[i-1, 1]), np.cos(dh[i-1, 1]),
                     np.cos(dh[i-1, 1]) * dh[i, 2]],
                    [0, 0, 0, 1]], dtype='float')

    return m_T


def main():

    inputs = np.radians(np.array([0, 0, 0, 0, 0, 0]))  # Joint angles, but could be R or P

    # a_m-1, alpha_m-1, d_m, theta_m
    dh = np.array([[0, np.pi/2, 0.1807, inputs[0]],
                   [-0.6127, 0, 0, inputs[1]],
                   [-0.57155, 0, 0, inputs[2]],
                   [0, np.pi/2, 0.17415, inputs[3]],
                   [0, -np.pi/2, 0.11985, inputs[4]],
                   [0, 0, 0.11655, inputs[5]]], dtype='float')
    ax = init_axis()

    u_0 = CoordinateAxis(position=[0, 0, 0], orientation=[0, 0, 0])
    u_1 = CoordinateAxis(position=[0, 0, 0], orientation=[0, 0, 0])
    u_2 = CoordinateAxis(position=[0, 0, 0], orientation=[0, 0, 0])
    u_3 = CoordinateAxis(position=[0, 0, 0], orientation=[0, 0, 0])
    u_4 = CoordinateAxis(position=[0, 0, 0], orientation=[0, 0, 0])
    u_5 = CoordinateAxis(position=[0, 0, 0], orientation=[0, 0, 0])
    u_6 = CoordinateAxis(position=[0, 0, 0], orientation=[0, 0, 0])

    m_01 = link_transformation(dh, 0)
    m_12 = link_transformation(dh, 1)
    m_23 = link_transformation(dh, 2)
    m_34 = link_transformation(dh, 3)
    m_45 = link_transformation(dh, 4)
    m_56 = link_transformation(dh, 5)

    m_02 = m_01 @ m_12
    m_03 = m_01 @ m_12 @ m_23
    m_04 = m_01 @ m_12 @ m_23 @ m_34
    m_05 = m_01 @ m_12 @ m_23 @ m_34 @ m_45
    m_06 = m_01 @ m_12 @ m_23 @ m_34 @ m_45 @ m_56

    u_1.transform_frame(m_01)
    u_2.transform_frame(m_02)
    u_3.transform_frame(m_03)
    u_4.transform_frame(m_04)
    u_5.transform_frame(m_05)
    u_6.transform_frame(m_06)

    u_0.setup_plot(ax)
    u_1.setup_plot(ax)
    u_2.setup_plot(ax)
    u_3.setup_plot(ax)
    u_4.setup_plot(ax)
    u_5.setup_plot(ax)
    u_6.setup_plot(ax)

    joint_pos = np.concatenate((u_0.origin, u_1.origin, u_2.origin, u_3.origin, u_4.origin, u_5.origin, u_6.origin))
    print(joint_pos)
    plt.plot(*zip(*joint_pos), color='grey', linewidth=10, marker='o', markersize=10, mfc='deepskyblue')
    plt.show()


if __name__ == "__main__":
    main()
