# Author: ivantregear
# Date Created: 19/01/2023
# Description: 

# When I wrote this code only got I and knew how it worked.
# Now only god knows it.

import matplotlib.pyplot as plt
import numpy as np


def main():
    ja = np.array([0, 0, 0])  # Initial joint positions
    jb = np.array([np.pi/2, 2, 3])  # Final joint positions

    a = np.array([0.2, 0.1, 0.1])  # Max possible joint accelerations

    ts = np.sqrt((jb - ja) / a)
    V = a * ts
    tf = 2 * ts

    tb = (ja - jb + V) / V * tf

    t = np.array([np.linspace(0, tf[0], 100), np.linspace(0, tf[1], 100), np.linspace(0, tf[2], 100)])

    for i in range(len(ja)):
        theta = np.piecewise(t[i], [(t[i] >= 0) * (t[i] <= ts[i]),
                                 (t[i] > ts[i]) * (t[i] <= tf[i])],
                             [lambda t: ja[i] + V[i] / (2 * tb[i]) * t ** 2,
                              lambda t: jb[i] - a[i] * tf[i] ** 2 / 2 + a[i] * tf[i] * t - a[i] * t ** 2 / 2])
        plt.plot(t[i], theta)

    plt.show()


if __name__ == "__main__":
    main()
