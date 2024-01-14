# Author: ivantregear
# Date Created: 19/01/2023
# Description: 

# When I wrote this code only got I and knew how it worked.
# Now only god knows it.

import matplotlib.pyplot as plt
import numpy as np


def main():
    ja = np.array([0, 0, 0])  # Initial joint positions
    jb = np.array([2, np.pi/2, np.pi/2])  # Final joint positions

    acc = np.array([1.5, 2, 1])  # Additional acceleration factors

    tf = 2  # Total time
    th = tf/2  # Half-time

    a = (jb - ja) / th**2 * acc  # Joint accelerations

    tb = th - np.sqrt(a**2 * th**2 - a*(jb - ja)) / a  # Blend times

    V = a * tb  # Constant velocities

    t = np.linspace(0, tf, 100)  # Time domain for plotting

    for i in range(len(ja)):
        theta = np.piecewise(t, [(t >= 0)*(t <= tb[i]),
                                 (t > tb[i])*(t <= tf - tb[i]),
                                 (t > tf - tb[i])*(t <= tf)],
                             [lambda t: ja[i] + V[i]/(2*tb[i])*t**2, lambda t: (jb[i] + ja[i] - V[i]*tf)/2 + V[i]*t,
                              lambda t: jb[i] - a[i]*tf**2/2 + a[i]*tf*t - a[i]*t**2/2])
        plt.plot(t, theta)

    plt.show()


if __name__ == "__main__":
    main()
