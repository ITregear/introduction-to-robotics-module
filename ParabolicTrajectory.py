# Author: ivantregear
# Date Created: 19/01/2023
# Description: 

# When I wrote this code only got I and knew how it worked.
# Now only god knows it.

import numpy as np
import matplotlib.pyplot as plt


def blended_parabolic_motion(pos, accel, t_d, n):

    def sign(num):
        if num == 0:
            return 1
        else:
            return num / abs(num)

    n_points = len(pos)
    n_joints = len(pos[0])

    inputs = np.zeros([(n_points - 1)*n, n_joints])

    for joint in range(n_joints):

        time_array = []

        # Specified
        theta = pos[:, joint]  # Points
        # Specified
        acc = accel[joint]

        # Not specified
        t_b = np.zeros(n_points)  # Blend time at each point
        t_l = np.zeros(n_points - 1)  # Total time between points
        vel = np.zeros(n_points - 1)  # Constant velocity between points
        theta_dot = np.zeros(n_points)  # Velocities at each point

        # First segment
        acc[0] = sign(theta[1] - theta[0]) * abs(acc[0])
        t_b[0] = t_d[0] - np.sqrt(t_d[0] ** 2 - 2 * (theta[1] - theta[0]) / acc[0])
        vel[0] = (theta[1] - theta[0]) / (t_d[0] - 0.5 * t_b[0])

        # Last segment
        acc[-1] = sign(theta[-2] - theta[-1]) * abs(acc[-1])  # Notes say theta[-2] - theta[-1]
        t_b[-1] = t_d[-1] - np.sqrt(t_d[-1] ** 2 - 2 * (theta[-2] - theta[-1]) / acc[-1])
        vel[-1] = (theta[-1] - theta[-2]) / (t_d[-1] - 0.5 * t_b[-1])

        # Via points
        for i in range(1, n_points - 2):
            vel[i] = (theta[i + 1] - theta[i]) / t_d[i]
        for i in range(0, n_points - 2):
            acc[i+1] = sign(vel[i + 1] - vel[i]) * abs(acc[i+1])
        for i in range(0, n_points - 2):
            t_b[i+1] = 0.5 * (vel[i + 1] - vel[i]) / acc[i+1]
        for i in range(1, n_points - 2):
            t_l[i] = t_d[i] - 0.5 * t_b[i] - 0.5 * t_b[i+1]

        # t_l for 1st and last points
        t_l[0] = t_d[0] - t_b[0] - 0.5 * t_b[1]
        t_l[-1] = t_d[-1] - t_b[-1] - 0.5 * t_b[-2]

        # print("Points:\t", theta)
        # print("Acc:\t", acc)
        # print("T_d:\t", t_d)
        # print()
        # print("Vel:\t", vel)
        # print("T_b:\t", t_b)
        # print("T_l:\t", t_l)

        interp = np.array([theta[0]], dtype='float64')
        theta_0 = theta[0]
        theta_1, theta_2 = 0, 0

        for i in range(0, n_points - 1):
            theta_dot[i+1] = t_b[i+1] * acc[i+1] + vel[i]

            time = np.linspace(0, t_d[i], n)

            for t in time:
                if t < t_b[i]:
                    interp = np.append(interp, theta_0 + acc[i] * t**2 / 2 + theta_dot[i]*t)
                    theta_1 = interp[-1]
                if t_b[i] <= t <= t_d[i] - t_b[i+1]:
                    interp = np.append(interp, theta_1 + (t - t_b[i])*vel[i])
                    theta_2 = interp[-1]
                if t_d[i] - t_b[i+1] < t <= t_d[i]:
                    interp = np.append(interp, theta_2 + acc[i+1]*(t + t_b[i+1] - t_d[i])**2/2 + vel[i] *
                                       (t + t_b[i+1] - t_d[i]))
                    theta_0 = interp[-1]

            time_array += list(time + np.sum(t_d[:i]))
        inputs[:, joint] = interp[1:]

    time_array = np.array(time_array)
    return time_array, inputs


def main():
    np.set_printoptions(formatter={'float': lambda x: "{0:0.3f}".format(x)})

    t1 = np.pi/4
    d3 = 0.8

    inputs = np.array([[0, np.pi / 2, 0.2, np.pi / 2],
                            [-2.977, 0, 0.75, 0]])

    accelerations = np.array([np.ones(len(inputs)) * 5,
                              np.ones(len(inputs)) * 5,
                              np.ones(len(inputs)) * 15,
                              np.ones(len(inputs)) * 5])

    times = np.array([3])

    fig, ax = plt.subplots()
    ax2 = ax.twinx()

    ax.set_title("Joint Space Trajectory Planning using LSPB Interpolation")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Joint Value [rad]")
    ax2.set_ylabel("Joint Value [m]")
    ax.grid()
    t, pos = blended_parabolic_motion(inputs, accelerations, times, n=500)
    ax.scatter(np.append([0], np.cumsum(times)), inputs[:, 0], label=r"x_1", c='r', marker='x')
    ax.scatter(np.append([0], np.cumsum(times)), inputs[:, 1], label=r"y_2", c='g', marker='x')
    ax.scatter(np.append([0], np.cumsum(times)), inputs[:, 2], label=r"$\theta$", c='b', marker='x')
    ax.plot(t, pos[:, 0], c='r')
    ax.plot(t, pos[:, 1], c='g')
    ax.plot(t, pos[:, 2], c='b')
    ax.legend()
    plt.show()


if __name__ == "__main__":
    main()
