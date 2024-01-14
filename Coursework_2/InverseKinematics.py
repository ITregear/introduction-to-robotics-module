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


def align_yaxis_np(ax1, ax2):
    """Align zeros of the two axes, zooming them out by same ratio"""
    axes = np.array([ax1, ax2])
    extrema = np.array([ax.get_ylim() for ax in axes])
    tops = extrema[:,1] / (extrema[:,1] - extrema[:,0])
    # Ensure that plots (intervals) are ordered bottom to top:
    if tops[0] > tops[1]:
        axes, extrema, tops = [a[::-1] for a in (axes, extrema, tops)]

    # How much would the plot overflow if we kept current zoom levels?
    tot_span = tops[1] + 1 - tops[0]

    extrema[0,1] = extrema[0,0] + tot_span * (extrema[0,1] - extrema[0,0])
    extrema[1,0] = extrema[1,1] + tot_span * (extrema[1,0] - extrema[1,1])
    [axes[i].set_ylim(*extrema[i]) for i in range(2)]


def main():
    np.set_printoptions(formatter={'float': lambda x: "{0:0.3f}".format(x)})

    # Defining Differential Drive Robot
    l = 0.5
    r = 0.15
    d = 0.25

    mobile_robot = DifferentialDriveRobot(l, r, d)

    start_pos, end_pos, target_pos, path = extract_image_parameters("ImageParametersimg_2.txt")

    theta_arr = np.zeros((len(path), 1))
    for i in range(1, len(path)-1):
        theta = np.arctan2(-(path[i, 1] - path[i - 1, 1]), (path[i, 0] - path[i - 1, 0]) + 0.000001)
        theta_arr[i - 1, 0] = theta

    theta_arr[-2, 0] = theta_arr[-3, 0]
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

    fig, ax = plt.subplots(nrows=4, ncols=1)
    ax1 = ax[0].twinx()
    ax[0].set_title("Cartesian Positions vs Time")
    l1, = ax[0].plot(times, ip[:, 0], label="X")
    l2, = ax[0].plot(times, ip[:, 1], label="Y", c='orange')
    l3, = ax1.plot(times, ip[:, 2], label=r"$\theta$", c='green')
    ax[0].set_xlabel("Time [s]")
    ax[0].set_ylabel("Position [m]")
    ax1.set_ylabel("Position [rad]")
    ax[0].grid()
    ax[0].set_xlim([0, times[-1]])
    ax[0].legend(handles=[l1, l2, l3], loc='center right')
    align_yaxis_np(ax[0], ax1)
    plt.draw()

    ax2 = ax[1].twinx()
    ax[1].set_title("Cartesian Velocities vs Time")
    l1, = ax[1].plot(times[:-1], ip_dot[:, 0], label=r"$\dot{x}$")
    l2, = ax[1].plot(times[:-1], ip_dot[:, 1], label=r"$\dot{y}$", c='orange')
    l3, = ax2.plot(times[:-1], ip_dot[:, 2], label=r"$\dot{\theta}$", c='green')
    ax[1].set_xlabel("Time [s]")
    ax[1].set_ylabel("Velocity [m/s]")
    ax2.set_ylabel("Angular Velocity [rad/s]")
    ax[1].grid()
    ax[1].set_xlim([0, times[-1]])
    ax[1].legend(handles=[l1, l2, l3])
    align_yaxis_np(ax[1], ax2)
    plt.draw()

    ax[2].set_title("Wheel Velocities vs Time")
    ax[2].plot(times[:-1], phi_dot[:, 0], label=r"$\dot{\phi}_l$")
    ax[2].plot(times[:-1], phi_dot[:, 1], label=r"$\dot{\phi}_r$", c='orange')
    ax[2].set_xlabel("Time [s]")
    ax[2].set_ylabel("Angular Velocity [rad/s]")
    ax[2].grid()
    ax[2].set_xlim([0, times[-1]])
    ax[2].legend()
    plt.draw()

    ax[3].set_title("Wheel Positions vs Time")
    ax[3].plot(times[:-1], phi[:, 0]-phi[0, 0], label=r"$\phi_l$")
    ax[3].plot(times[:-1], phi[:, 1]-phi[0, 1], label=r"$\phi_r$", c='orange')
    ax[3].set_xlabel("Time [s]")
    ax[3].set_ylabel("Angular Position [rad]")
    ax[3].grid()
    ax[3].set_xlim([0, times[-1]])
    ax[3].legend()
    plt.draw()

    plt.show()


if __name__ == "__main__":
    main()
