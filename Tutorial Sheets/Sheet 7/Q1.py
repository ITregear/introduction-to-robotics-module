# Author: ivantregear
# Date Created: 24/02/2023
# Description: 

# When I wrote this code only God I and knew how it worked.
# Now only god knows it.

import numpy as np
import matplotlib.pyplot as plt


def main():

    w = 0.05
    h = 0.03
    nx = 1200
    ny = 1000

    dw = w/nx
    dh = h/ny

    f = 0.018

    cube_coords = np.array([[1, -1, 1, 1],
                            [1, -1, -1, 1],
                            [1, 1, 1, 1],
                            [1, 1, -1, 1],
                            [-1, -1, 1, 1],
                            [-1, -1, -1, 1],
                            [-1, 1, 1, 1],
                            [-1, 1, -1, 1]])

    cm = np.array([[f/dw, 0, nx/2, 0],
                   [0, f/dh, ny/2, 0],
                   [0, 0, 1, 0]])

    T_0_c = np.array([[1, 0, 0, 0],
                      [0, 1, 0, 0],
                      [0, 0, 1, -10],
                      [0, 0, 0, 1]])

    p_ = np.zeros([8, 3])
    u_v = np.zeros([8, 2])

    fig, ax = plt.subplots()
    line, = ax.plot([0, 1], [0, 1], 's')
    plt.xlim([0, 1200])
    plt.ylim([0, 1000])
    plt.grid()

    alpha = np.linspace(0, 2*np.pi, 100)
    ur = np.array([0, 1, 0])

    for j in range(len(alpha)):
        a = alpha[j]

        R_0_c = np.array([[ur[0]**2*(1-np.cos(a))+np.cos(a), ur[0]*ur[1]*(1-np.cos(a))-ur[2]*np.sin(a), ur[0]*ur[2]*(1-np.cos(a))+ur[1]*np.sin(a), 0],
                          [ur[0]*ur[1]*(1-np.cos(a))+ur[2]*np.sin(a), ur[1]**2*(1-np.cos(a))+np.cos(a), ur[1]*ur[2]*(1-np.cos(a))-ur[0]*np.sin(a), 0],
                          [ur[0]*ur[2]*(1-np.cos(a))-ur[1]*np.sin(a), ur[1]*ur[2]*(1-np.cos(a))+ur[0]*np.sin(a), ur[2]**2*(1-np.cos(a))+np.cos(a), 0],
                          [0, 0, 0, 1]])

        T_0_c_j = np.matmul(R_0_c, T_0_c)

        for i in range(len(cube_coords)):
            p_[i, :] = np.matmul(np.matmul(cm, np.linalg.inv(T_0_c_j)), cube_coords[i, :])
            u_v[i, :] = np.array([p_[i, 0]/p_[i, 2], p_[i, 1]/p_[i, 2]])

        line.set_data(*zip(*u_v))
        plt.pause(0.01)

    plt.show()


if __name__ == "__main__":
    np.set_printoptions(formatter={'float': lambda x: "{0:0.3f}".format(x)})
    main()
