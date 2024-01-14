# Author: ivantregear
# Date Created: 07/11/2022
# Description: Serial manipulator class
# Goal: Take nDOF serial manipulator, and plot motion based on joint inputs
# Should take both R and P joints
# Should be able to plot relatively quickly (>30fps)
# Should be able to automatically reconfigure for nDOF
# WIP
# TODO:
# Allow origin to move

# When I wrote this code only got I and knew how it worked.
# Now only god knows it.


import numpy as np
import matplotlib.pyplot as plt
from FrameTransformation import CoordinateAxis


class SerialManipulator:
    def __init__(self, dh, n_links, axis, base_pos, base_ori):
        self.n_link = n_links
        self.n_frame = self.n_link + 1
        self.dh_func = dh
        self.inputs = np.zeros(self.n_link)

        self.end_effector_position = base_pos
        self.base_to_end_transformation = np.eye(4)
        self.m_b = self.create_base_frame(base_pos, base_ori)

        self.frames = [CoordinateAxis(0.2) for _ in range(self.n_frame)]
        self.axis = axis
        self.joint_pos = []
        self.dh = self.dh_func(self.inputs)
        self.show_coordinates = True
        self.line, = self.axis.plot3D([], [], [], color='grey', linewidth=10, marker='o', markersize=10,
                                      mfc='deepskyblue')
        self.plot_setup_flag = False

    def reset_frames(self):
        [frame.reset() for frame in self.frames]

    def create_base_frame(self, p, o):

        m_t = np.array([[np.cos(o[1]) * np.cos(o[2]),
                              np.sin(o[0]) * np.sin(o[1]) * np.cos(o[2]) - np.cos(o[0]) * np.sin(o[2]),
                              np.cos(o[0]) * np.sin(o[1]) * np.cos(o[2]) + np.sin(o[0]) * np.sin(o[2]), p[0]],
                             [np.cos(o[1]) * np.cos(o[2]),
                              np.sin(o[0]) * np.sin(o[1]) * np.sin(o[2]) + np.cos(o[0]) * np.cos(o[2]),
                              np.cos(o[0]) * np.sin(o[1]) * np.sin(o[2]) - np.sin(o[0]) * np.cos(o[2]), p[1]],
                             [-np.sin(o[1]), np.sin(o[0]) * np.cos(o[1]), np.cos(o[0]) * np.cos(o[1]), p[2]],
                             [0, 0, 0, 1]])

        return m_t

    def update_position(self):

        self.reset_frames()

        self.dh = self.dh_func(self.inputs)

        m_i = self.m_b

        self.frames[0].transform_frame(m_i)

        for i in range(1, self.n_frame):
            m_i = m_i @ self.link_transformation(i - 1)

            self.frames[i].transform_frame(m_i)

        self.base_to_end_transformation = m_i

        self.end_effector_position = m_i[:-1, -1]

        self.joint_pos = np.concatenate([frame.origin for frame in self.frames])

    def plot_position(self):
        if not self.plot_setup_flag:
            if self.show_coordinates:
                for frame in self.frames:
                    frame.setup_plot(self.axis)
            self.plot_setup_flag = True
        else:
            if self.show_coordinates:
                for frame in self.frames:
                    frame.update_plot()

        self.line.set_data_3d(*zip(*self.joint_pos))
        plt.pause(0.001)

    def link_transformation(self, i):

        m_t = np.array([[np.cos(self.dh[i, 3]), -np.sin(self.dh[i, 3]), 0, self.dh[i, 1]],
                        [np.sin(self.dh[i, 3]) * np.cos(self.dh[i, 0]), np.cos(self.dh[i, 3]) * np.cos(self.dh[i, 0]),
                         -np.sin(self.dh[i, 0]), -self.dh[i, 2] * np.sin(self.dh[i, 0])],
                        [np.sin(self.dh[i, 3]) * np.sin(self.dh[i, 0]), np.cos(self.dh[i, 3]) * np.sin(self.dh[i, 0]),
                         np.cos(self.dh[i, 0]), self.dh[i, 2] * np.cos(self.dh[i, 0])],
                        [0, 0, 0, 1]], dtype='float')

        return m_t
