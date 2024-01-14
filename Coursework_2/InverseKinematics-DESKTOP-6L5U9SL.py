# Author: ivantregear
# Date Created: 20/03/2023
# Description: 

# When I wrote this code only God I and knew how it worked.
# Now only god knows it.

import numpy as np
import sys
import matplotlib.pyplot as plt
import time
from PathVisualisation import extract_image_parameters

sys.path.append("../")

from MobileRobot import DifferentialDriveRobot


def update_axis(ax, origin):
    ax.set_xlim([origin[0] - 1, origin[0] + 1])
    ax.set_ylim([origin[1] - 1, origin[1] + 1])


def find_closest(array, value):
    return (np.abs(array - value)).argmin()


def main():
    np.set_printoptions(formatter={'float': lambda x: "{0:0.3f}".format(x)})

    # Defining Differential Drive Robot
    l = 0.5
    r = 0.15
    d = 0.25

    mobile_robot = DifferentialDriveRobot(l, r, d)

    start_pos, end_pos, target_pos, path = extract_image_parameters("ImageParametersimg.txt")

    theta_arr = np.zeros((len(path), 1))
    for i in range(1, len(path)):
        theta = np.arctan2(-(path[i, 1] - path[i - 1, 1]), (path[i, 0] - path[i - 1, 0]) + 0.0001)
        theta_arr[i - 1, 0] = theta

    theta_arr[-1, 0] = theta_arr[-2, 0]
    ip = np.concatenate((path, theta_arr), axis=1)
    times = np.linspace(0, 10, len(ip))

    ip[:, 2] = np.unwrap(ip[:, 2])

    ip_dot = np.divide(np.diff(ip, axis=0).T, np.diff(times).T).T
    phi_dot = mobile_robot.inverse_kinematics(ip_dot, ip)

    phi = np.zeros([len(phi_dot), 2])
    for idx in range(0, len(phi_dot)):
        if idx == 0:
            phi[idx, :] = 0 + phi_dot[idx, :] * (times[idx + 1] - times[idx - 1])
        else:
            phi[idx, :] = phi[idx-1, :] + phi_dot[idx, :] * (times[idx+1] - times[idx-1])

    fig, ax = plt.subplots()
    ax2 = ax.twinx()
    plt.title("Cartesian Positions vs Time")
    l1, = ax.plot(times, ip[:, 0], label="X")
    l2, = ax.plot(times, ip[:, 1], label="Y", c='orange')
    l3, = ax2.plot(times, ip[:, 2], label=r"$\theta$", c='green')
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Position [m]")
    ax2.set_ylabel("Position [rad]")
    ax.grid()
    ax.set_xlim([0, times[-1]])
    ax.legend(handles=[l1, l2, l3])
    plt.draw()

    fig, ax = plt.subplots()
    ax2 = ax.twinx()
    plt.title("Cartesian Velocities vs Time")
    l1, = ax.plot(times[:-1], ip_dot[:, 0], label=r"$\dot{x}$")
    l2, = ax.plot(times[:-1], ip_dot[:, 1], label=r"$\dot{y}$", c='orange')
    l3, = ax2.plot(times[:-1], ip_dot[:, 2], label=r"$\dot{\theta}$", c='green')
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Velocity [m/s]")
    ax2.set_ylabel("Angular Velocity [rad/s]")
    ax.grid()
    ax.set_xlim([0, times[-1]])
    ax.legend(handles=[l1, l2, l3])
    plt.draw()

    fig, ax = plt.subplots()
    plt.title("Wheel Velocities vs Time")
    ax.plot(times[:-1], phi_dot[:, 0], label=r"$\dot{\phi}_l$")
    ax.plot(times[:-1], phi_dot[:, 1], label=r"$\dot{\phi}_r$", c='orange')
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Angular Velocity [rad/s]")
    ax.grid()
    ax.set_xlim([0, times[-1]])
    ax.legend()
    plt.draw()

    fig, ax = plt.subplots()
    plt.title("Wheel Positions vs Time")
    ax.plot(times[:-1], phi[:, 0]-phi[0, 0], label=r"$\phi_l$")
    ax.plot(times[:-1], phi[:, 1]-phi[0, 1], label=r"$\phi_r$", c='orange')
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Angular Position [rad]")
    ax.grid()
    ax.set_xlim([0, times[-1]])
    ax.legend()
    plt.draw()

    plt.show()


if __name__ == "__main__":
    main()
