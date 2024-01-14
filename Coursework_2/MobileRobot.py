# Author: ivantregear
# Date Created: 12/03/2023
# Description: 

# When I wrote this code only God I and knew how it worked.
# Now only god knows it.

import numpy as np
import sys
import matplotlib.pyplot as plt

sys.path.append("../")

from FrameTransformation import CoordinateAxis


class DifferentialDriveRobot:
    def __init__(self, l, r, d, axis=None):
        self.l = l  # Wheelbase
        self.r = r  # Wheel radius
        self.d = d  # Centre to wheelbase offset
        self.axis = axis

        self.Tp = np.array([[1 / self.r, 0, self.l / self.r],
                            [1 / self.r, 0, -self.l / self.r]])
        self.phi = np.array(2)
        self.m_i = np.eye(4)

        self.plot_setup_flag = False

        self.origin_frame = CoordinateAxis(0.2)
        self.direction_matrix = np.eye(3)

        if self.axis:
            self.body, = self.axis.plot3D([], [], [], color='black')
            self.left_wheel, = self.axis.plot3D([], [], [], color='black')
            self.right_wheel, = self.axis.plot3D([], [], [], color='black')
            self.caster, = self.axis.plot3D([], [], [], color='black')
            self.left_wheel_marker, = self.axis.plot3D([], [], [], color='red', linewidth=5)
            self.right_wheel_marker, = self.axis.plot3D([], [], [], color='red', linewidth=5)

        self.ip = np.zeros(3)
        self.ip_dot = np.zeros(3)
        self.phi = np.zeros(2)

    def plot_position(self):

        if not self.plot_setup_flag:
            self.origin_frame.setup_plot(self.axis)
            self.plot_setup_flag = True
        else:
            self.origin_frame.update_plot()

        def draw_circle(c, r, a, b, theta_start):

            a = a / np.linalg.norm(a)
            b = b / np.linalg.norm(a)

            theta = np.linspace(theta_start, theta_start + 2 * np.pi, 20)
            x = c[0] + r * np.cos(theta) * b[0] + r * np.sin(theta) * a[0]
            y = c[1] + r * np.cos(theta) * b[1] + r * np.sin(theta) * a[1]
            z = c[2] + r * np.cos(theta) * b[2] + r * np.sin(theta) * a[2]

            return x, y, z

        def rotate_artists(x1, y1, z1, r=self.m_i):
            points = np.row_stack([x1, y1, z1, np.ones(len(x1))])
            points_r = np.matmul(r, points)
            return points_r[0], points_r[1], points_r[2]

        x, y, z = draw_circle(np.array([self.d, 0, 0]), 1.25 * self.l, np.array([1, 0, 0]), np.array([0, 1, 0]), 0)
        x1, y1, z1 = rotate_artists(x, y, z)
        self.body.set_data_3d(x1, y1, z1)

        x, y, z = draw_circle(np.array([0, self.l / 2, 0]), self.r, np.array([1, 0, 0]), np.array([0, 0, 1]),
                              self.phi[0])
        right_marker_pos = np.array([[0, self.l / 2, 0], [x[0], y[0], z[0]]])
        x1, y1, z1 = rotate_artists(x, y, z)
        xrmp, yrmp, zrmp = rotate_artists(*zip(*right_marker_pos))
        self.right_wheel_marker.set_data_3d(xrmp, yrmp, zrmp)
        self.right_wheel.set_data_3d(x1, y1, z1)

        x, y, z = draw_circle(np.array([0, -self.l / 2, 0]), self.r, np.array([1, 0, 0]), np.array([0, 0, 1]),
                              self.phi[1])
        left_marker_pos = np.array([[0, -self.l / 2, 0], [x[0], y[0], z[0]]])
        x1, y1, z1 = rotate_artists(x, y, z)
        xlmp, ylmp, zlmp = rotate_artists(*zip(*left_marker_pos))
        self.left_wheel_marker.set_data_3d(xlmp, ylmp, zlmp)
        self.left_wheel.set_data_3d(x1, y1, z1)

        x, y, z = draw_circle(np.array([0, 0, 0]), self.r / 2, np.array([1, 0, 0]),
                              np.array([0, 0, 1]),
                              0)
        x, y, z = rotate_artists(x, y, z, r=np.array([[np.cos(-self.ip_dot[2]), -np.sin(-self.ip_dot[2]), 0, 0],
                                                      [np.sin(-self.ip_dot[2]), np.cos(-self.ip_dot[2]), 0, 0],
                                                      [0, 0, 1, 0],
                                                      [0, 0, 0, 1]]))
        x += 2 * self.d
        z += -self.r/2
        x1, y1, z1 = rotate_artists(x, y, z)
        self.caster.set_data_3d(x1, y1, z1)

    def update_position(self, ip):
        self.ip = ip
        self.m_i = np.array([[np.cos(ip[2]), np.sin(ip[2]), 0, ip[0]],
                             [-np.sin(ip[2]), np.cos(ip[2]), 0, ip[1]],
                             [0, 0, 1, self.r / 2],
                             [0, 0, 0, 1]])

        self.origin_frame.reset()
        self.origin_frame.transform_frame(self.m_i)

    def inverse_kinematics(self, ip_dot, ip):
        phi = []
        for idx in range(len(ip) - 1):
            phi += [self.Tp @ self.r_0_i(ip[idx, 2]) @ ip_dot[idx].T]
        self.phi = np.array(phi)
        return self.phi

    def r_0_i(self, theta):
        return np.array([[np.cos(theta), -np.sin(theta), 0],
                         [np.sin(theta), np.cos(theta), 0],
                         [0, 0, 1]])


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


def update_axis(ax, origin):
    ax.set_xlim([origin[0] - 1, origin[0] + 1])
    ax.set_ylim([origin[1] - 1, origin[1] + 1])


def find_closest(array, value):
    return (np.abs(array - value)).argmin()


def main():
    1


if __name__ == "__main__":
    main()
