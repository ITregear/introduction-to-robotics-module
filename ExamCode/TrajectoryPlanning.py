# Author: ivantregear
# Date Created: 11/04/2023
# Description: 

# When I wrote this code only God I and knew how it worked.
# Now only god knows it.

import numpy as np
import matplotlib.pyplot as plt


def cubic_polynomial(theta, ti):
    n_eq = len(theta) - 1
    b = np.zeros(4 * n_eq)
    a = np.zeros([4 * n_eq, 4 * n_eq])

    # Zero initial velocity condition
    a[0, 1] = 1
    b[0] = 0

    # Zero final velocity condition
    a[-1, 4 * (n_eq - 1) + 1] = 1
    a[-1, 4 * (n_eq - 1) + 2] = 2 * ti[-1]
    a[-1, 4 * (n_eq - 1) + 3] = 3 * ti[-1]**2
    b[-1] = 0

    for i in range(n_eq):
        # First positional condition
        a[4 * i + 1, 4 * i] = 1
        b[4 * i + 1] = theta[i]

        # Second positional condition
        a[4 * i + 2, 4 * i] = 1
        a[4 * i + 2, 4 * i + 1] = ti[i]
        a[4 * i + 2, 4 * i + 2] = ti[i]**2
        a[4 * i + 2, 4 * i + 3] = ti[i]**3
        b[4 * i + 2] = theta[i + 1]

    for i in range(n_eq-1):
        # Acceleration condition
        a[4 * i + 3, 4 * i + 2] = 2
        a[4 * i + 3, 4 * i + 3] = 6 * ti[i]
        a[4 * i + 3, 4 * (i+1) + 2] = -2

        # Intermediate velocity condition
        a[4 * i + 4, 4 * i + 1] = 1
        a[4 * i + 4, 4 * i + 2] = 2 * ti[i+1]
        a[4 * i + 4, 4 * i + 3] = 3 * ti[i+1]**2
        a[4 * i + 4, 4 * (i+1) + 1] = -1

    coefficients = np.linalg.solve(a, b)

    return coefficients


def cubic_polynomial_plot(theta, ti, a):
    t_cum = np.cumsum(ti)

    t_cum = np.insert(t_cum, 0, 0)

    xc, yc = np.zeros(1), np.zeros(1)

    for i in range(len(theta) - 1):
        t = np.linspace(t_cum[i], t_cum[i+1], 100)
        y = a[4 * i] + a[4 * i + 1] * (t-t_cum[i]) + a[4 * i + 2] * (t - t_cum[i])**2 + a[4 * i + 3] * (t - t_cum[i])**3

        xc = np.append(xc, t)
        yc = np.append(yc, y)

    xc = np.delete(xc, 0)
    yc = np.delete(yc, 0)

    vel = np.gradient(yc, xc)
    acc = np.gradient(vel, xc)

    plt.scatter(t_cum, theta)
    plt.plot(xc, yc, label="")
    plt.plot(xc, vel, label="Velocity")
    plt.plot(xc, acc, label="Acceleration")
    plt.legend()


def blended_parabolic(pos, accel, t_d, n):

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
                elif t_b[i] <= t <= t_d[i] - t_b[i+1]:
                    interp = np.append(interp, theta_1 + (t - t_b[i])*vel[i])
                    theta_2 = interp[-1]
                elif t_d[i] - t_b[i+1] < t <= t_d[i]:
                    interp = np.append(interp, theta_2 + acc[i+1]*(t + t_b[i+1] - t_d[i])**2/2 + vel[i] *
                                       (t + t_b[i+1] - t_d[i]))
                    theta_0 = interp[-1]

            time_array += list(time + np.sum(t_d[:i]))
        inputs[:, joint] = interp[1:]

    time_array = np.array(time_array)
    return time_array, inputs


def blended_parabolic_plot(inputs, times, t, pos):
    for i in range(len(inputs[0])):
        plt.scatter(np.append([i], np.cumsum(times)), inputs[:, i], label='Points')
        plt.plot(t, pos[:, i], label='Blended Parabolic')


def main():

    theta = np.array([[30],
                      [75]])

    ti = np.array([5])

    coeffs = cubic_polynomial(theta, ti)
    print(coeffs)
    cubic_polynomial_plot(theta, ti, coeffs)

    plt.legend()
    plt.show()


if __name__ == "__main__":
    main()
