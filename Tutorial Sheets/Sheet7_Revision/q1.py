# Author: ivantregear
# Date Created: 18/04/2023
# Description: 

# When I wrote this code only God I and knew how it worked.
# Now only god knows it.

import numpy as np
import matplotlib.pyplot as plt
import time


def camera_transformation(nx, ny, w, h, f, cam_pos, cam_angle, image_pos):
    dw = w / nx
    dh = h / ny

    u0 = round(nx / 2)
    v0 = round(ny / 2)

    s = np.sin
    c = np.cos

    a = cam_angle[0]
    b = cam_angle[1]
    g = cam_angle[2]

    r_0_c = np.array([[c(b)*c(g), s(a)*s(b)*c(g)-c(a)*s(g), c(a)*s(b)*c(g)+s(a)*s(g)],
                      [c(b)*s(g), s(a)*s(b)*s(g)+c(a)*c(g), c(a)*s(b)*s(g)-s(a)*c(g)],
                      [-s(b), s(a)*c(b), c(a)*c(b)]])

    t_0_c = np.vstack((np.hstack((r_0_c, np.array([[cam_pos[0]], [cam_pos[1]], [cam_pos[2]]]))), np.array([[0, 0, 0, 1]])))

    cm = np.array([[f / dw, 0, u0, 0],
                   [0, f / dh, v0, 0],
                   [0, 0, 1, 0]])

    cube_points_hom = np.hstack((image_pos, np.ones((len(image_pos), 1)))).T

    p_tilda = cm @ np.linalg.inv(t_0_c) @ cube_points_hom

    p = np.array([p_tilda[0] / p_tilda[2], p_tilda[1] / p_tilda[2]])

    return p


def main():
    np.set_printoptions(suppress=True)

    fig = plt.figure(figsize=(12, 6))
    ax1 = fig.add_subplot(1, 2, 1)
    ax2 = fig.add_subplot(1, 2, 2, projection='3d')

    pixels, = ax1.plot([], [], 's')

    f = 0.075  # Focal length, m

    n_x = 1200  # Horizontal pixel count
    n_y = 1000  # Vertical pixel count

    w = 0.05  # Sensor width, m
    h = 0.03  # Sensor height, m

    cube_points = np.array([[-1, 1, -1],
                            [1, 1, -1],
                            [-1, -1, -1],
                            [1, -1, -1],
                            [-1, 1, 1],
                            [1, 1, 1],
                            [-1, -1, 1],
                            [1, -1, 1]])

    theta = np.linspace(0, 2*np.pi, 100)

    r = 10  # radius of circular path, m
    elev = np.radians(0)  # radius of elevated path, deg

    for t in theta:

        cam_pos = np.array([r*np.cos(elev)*np.cos(t), r*np.cos(elev)*np.sin(t), -r*np.sin(elev)])
        cam_angle = np.array([0, 0, 0])
        print(cam_pos)

        p = camera_transformation(n_x, n_y, w, h, f, cam_pos, cam_angle, cube_points)
        pixels.set_data(*p)
        plt.pause(0.01)

    ax1.axis('equal')
    # ax1.set(xlim=(0, n_x), ylim=(n_y, 0))
    plt.show()


if __name__ == "__main__":
    # main()

    n_x = 800
    n_y = 600
    f = 1.8

    w = 1
    h = 1

    p = camera_transformation(n_x, n_y, )
