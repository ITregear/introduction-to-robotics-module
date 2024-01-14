# Author: ivantregear
# Date Created: 15/03/2023
# Description: 

# When I wrote this code only God I and knew how it worked.
# Now only god knows it.

import numpy as np
import sys
import matplotlib.pyplot as plt
import time
import keyboard

sys.path.append("../")

from MobileRobot import DifferentialDriveRobot
from Robot import SerialManipulator
from ParabolicTrajectory import blended_parabolic_motion


def init_axis():
    ax = plt.figure().add_subplot(projection='3d')
    ax.set_zlim([0, 2])
    # ax.set_xlim([5, 35])
    # ax.set_ylim([-10, 20])
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_proj_type('ortho')
    # ax.zaxis.set_ticklabels([])
    ax.view_init(azim=-45, elev=35.264)  # azim=-45, elev=35.264
    plt.tight_layout()

    return ax


def update_axis(ax, origin):
    ax.set_xlim([origin[0] - 1, origin[0] + 1])
    ax.set_ylim([origin[1] - 1, origin[1] + 1])


def find_closest(array, value):
    return (np.abs(array - value)).argmin()


def extract_image_parameters(filename):
    with open(filename, 'r') as f:
        all_lines = f.readlines()

    start_pos = np.zeros(2)
    end_pos = np.zeros(2)
    target_pos = np.zeros(2)
    path = []

    path_flag = False
    for line in all_lines:

        if line.split(", ")[0] == "StartX":
            start_pos[0] = line.split(", ")[1]
        if line.split(", ")[0] == "StartY":
            start_pos[1] = line.split(", ")[1]
        if line.split(", ")[0] == "EndX":
            end_pos[0] = line.split(", ")[1]
        if line.split(", ")[0] == "EndY":
            end_pos[1] = line.split(", ")[1]
        if line.split(", ")[0] == "TargetX":
            target_pos[0] = line.split(", ")[1]
        if line.split(", ")[0] == "TargetY":
            target_pos[1] = line.split(", ")[1]

        if path_flag:
            path += [[float(line.split(",")[0]), float(line.split(",")[1])]]

        if line == "Path XY\n":
            path_flag = True

    path = np.array(path)

    return start_pos, end_pos, target_pos, path


def main():
    np.set_printoptions(formatter={'float': lambda x: "{0:0.3f}".format(x)})
    ax = init_axis()

    # Defining RRPR manipulator
    l1 = 0.4
    l2 = -0.2
    l3 = 0.1
    l4 = 0.1
    l5 = 0.25

    # alpha_i-1, a_i-1, d_i, theta_i
    def dh(inputs):
        d_h = np.array([[0, 0, l1, inputs[0]],
                        [np.pi / 2, l2, l3, inputs[1]],
                        [np.pi / 2, 0, inputs[2], 0],
                        [0, 0, 0, inputs[3]],
                        [0, l4, l5, np.pi / 2]], dtype='float')

        return d_h

    rrpr = SerialManipulator(dh, 5, ax, base_pos=np.array([0, 0, 0]), base_ori=np.array([0, 0, 0]))
    rrpr.show_coordinates = True

    # Defining Differential Drive Robot
    l = 0.5
    r = 0.15
    d = 0.25

    mobile_robot = DifferentialDriveRobot(l, r, d, ax)

    start_flag = False
    t0 = 0
    path_idx = 0
    rrpr_idx = 0

    start_pos, end_pos, target_pos, path = extract_image_parameters("ImageParametersimg_2.txt")

    theta_arr = np.zeros((len(path), 1))
    for i in range(1, len(path)-1):
        theta = np.arctan2(-(path[i, 1] - path[i - 1, 1]), (path[i, 0] - path[i - 1, 0]) + 0.000001)
        theta_arr[i - 1, 0] = theta

    theta_arr[-2, 0] = theta_arr[-3, 0]
    theta_arr[-1, 0] = theta_arr[-2, 0]
    ip = np.concatenate((path, theta_arr), axis=1)
    times = np.linspace(0, 30, len(ip))

    ip[:, 2] = np.unwrap(ip[:, 2])

    ip_dot = np.divide(np.diff(ip, axis=0).T, np.diff(times).T).T
    phi_dot = mobile_robot.inverse_kinematics(ip_dot, ip)

    phi = np.zeros([len(phi_dot), 2])
    for idx in range(0, len(phi_dot)):
        if idx == 0:
            phi[idx, :] = 0 + phi_dot[idx, :] * (times[idx + 1] - times[idx - 1])
        else:
            phi[idx, :] = phi[idx-1, :] + phi_dot[idx, :] * (times[idx+1] - times[idx-1])

    ax.scatter(start_pos[0], start_pos[1], 0, c='blue', s=15, marker='x')
    ax.scatter(end_pos[0], end_pos[1], 0, c='blue', s=15, marker='x')
    ax.scatter(target_pos[0], target_pos[1], 0, c='green', s=15, marker='x')
    ax.plot([target_pos[0], target_pos[0]], [target_pos[1], target_pos[1]], [0, 2], c='green', linestyle='--')
    path_line, = ax.plot3D([], [], [], c='red')
    cum_path = []

    t1 = np.arctan2((target_pos[1] - path[-1, 1]), (target_pos[0] - path[-1, 0])) - 1.08  # remove this awful 1.08
    d3 = np.sqrt((target_pos[1] - path[-1, 1])**2 + (target_pos[0] - path[-1, 0])**2)

    rrpr_inputs = np.array([[0, 3*np.pi/4, 0.2, -np.pi/2],
                            [t1, np.pi/2, d3-l2-l5, np.pi/2]])

    accelerations = np.array([np.ones(len(rrpr_inputs)) * 50,
                              np.ones(len(rrpr_inputs)) * 10,
                              np.ones(len(rrpr_inputs)) * 15,
                              np.ones(len(rrpr_inputs)) * 5])

    rrpr_times = np.array([3])

    t, pos = blended_parabolic_motion(rrpr_inputs, accelerations, rrpr_times, n=100)

    t += times[-1]

    while True:

        if keyboard.is_pressed('space'):
            t0 = time.perf_counter()
            start_flag = True

        if start_flag:
            current_time = time.perf_counter() - t0
            path_idx = find_closest(times, current_time)
            rrpr_idx = find_closest(t, current_time)

        mobile_robot.update_position(ip[path_idx])
        mobile_robot.phi = phi[min(path_idx, len(times)-2)]
        mobile_robot.ip_dot = ip_dot[min(path_idx, len(times)-2)]

        rrpr_origin = mobile_robot.m_i
        rrpr.inputs = pos[rrpr_idx]
        rrpr.m_b = rrpr_origin
        rrpr.update_position()
        rrpr.plot_position()

        cum_path += [np.array([ip[path_idx, 0], ip[path_idx, 1], 0])]

        mobile_robot.plot_position()
        path_line.set_data_3d(*zip(*cum_path))
        update_axis(ax, mobile_robot.m_i[:-1, -1])


if __name__ == "__main__":
    main()
