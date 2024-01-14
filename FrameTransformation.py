import numpy as np
import matplotlib.pyplot as plt


class CoordinateAxis:
    def __init__(self, scale):
        self.scale = scale
        self.coordinate_system = self.scale * np.array([[1, 0, 0],
                                                        [0, 1, 0],
                                                        [0, 0, 1]])

        self.T_ab = np.eye(4)
        self.origin = np.array([[0, 0, 0]])
        self.orientation = np.array([0, 0, 0])
        self.r = np.eye(3)

        self.transform_frame(self.T_ab)
        self.arrow = None
        self.ax = None

    def homogenous_transformation_matrix(self, rot_mat, translation):
        r_ab = rot_mat

        t_ab = translation.T

        self.T_ab = np.concatenate((r_ab, t_ab), axis=1)
        self.T_ab = np.concatenate((self.T_ab, np.array([[0, 0, 0, 1]])))

        return self.T_ab

    def rot_mat_axis(self, angle, axis):
        angle = np.radians(angle)

        self.r = np.array(
            [[axis[0] * axis[0] * (1 - np.cos(angle)) + np.cos(angle), axis[0] * axis[1] * (1 - np.cos(angle)) -
              axis[2] * np.sin(angle), axis[0] * axis[2] * (1 - np.cos(angle)) + axis[2] * np.sin(angle)],
             [axis[0] * axis[2] * (1 - np.cos(angle)) + axis[2] * np.sin(angle),
              axis[1] * axis[1] * (1 - np.cos(angle))
              + np.cos(angle), axis[1] * axis[2] * (1 - np.cos(angle)) - axis[1] * np.sin(angle)],
             [axis[0] * axis[2] * (1 - np.cos(angle)) - axis[1] * np.sin(angle),
              axis[1] * axis[2] * (1 - np.cos(angle))
              + axis[1] * np.sin(angle), axis[2] * axis[2] * (1 - np.cos(angle)) + np.sin(angle)]], dtype='float')

        return self.r

    def rot_mat_angles(self, rotation):
        alpha = np.radians(float(rotation[0]))
        beta = np.radians(float(rotation[1]))
        gamma = np.radians(float(rotation[2]))

        self.r = np.array([[np.cos(gamma) * np.cos(beta), np.cos(gamma) * np.sin(beta) * np.sin(alpha) - np.sin(gamma)
                            * np.cos(alpha),
                            np.cos(gamma) * np.sin(beta) * np.cos(alpha) + np.sin(gamma) * np.sin(alpha)],
                           [np.sin(gamma) * np.cos(beta), np.sin(gamma) * np.sin(beta) * np.sin(alpha) + np.cos(gamma)
                            * np.cos(alpha),
                            np.sin(gamma) * np.sin(beta) * np.cos(alpha) - np.cos(gamma) * np.sin(alpha)],
                           [-np.sin(beta), np.cos(beta) * np.sin(alpha), np.cos(beta) * np.cos(alpha)]], dtype='float')

        return self.r

    def transform_point(self, translation, rotation, point):
        p_b = point

        hom_T = self.homogenous_transformation_matrix(rotation, translation)

        p_a = np.matmul(hom_T, p_b)

        print("Transformed Point", p_a.T)

        return p_a

    def transform_frame(self, hom_t):
        self.coordinate_system = np.matmul(hom_t, np.concatenate((self.coordinate_system, [[1, 1, 1]])))
        self.coordinate_system = self.coordinate_system[:-1, :]

        self.origin = np.concatenate((self.origin, [[1]]), axis=1).T
        self.origin = np.matmul(hom_t, self.origin)
        self.origin = self.origin[:-1, :].T

    def setup_plot(self, axis):
        self.ax = axis

        self.arrow = self.ax.quiver(self.origin[:, 0], self.origin[:, 1], self.origin[:, 2],
                                    self.coordinate_system[0] - self.origin[:, 0],
                                    self.coordinate_system[1] - self.origin[:, 1],
                                    self.coordinate_system[2] - self.origin[:, 2], length=1,
                                    colors=['r', 'g', 'b', 'r', 'r', 'g', 'g', 'b', 'b'])

    def update_plot(self):
        if self.arrow:
            self.arrow.remove()
            self.arrow = self.ax.quiver(self.origin[:, 0], self.origin[:, 1], self.origin[:, 2],
                                        self.coordinate_system[0] - self.origin[:, 0],
                                        self.coordinate_system[1] - self.origin[:, 1],
                                        self.coordinate_system[2] - self.origin[:, 2], length=1,
                                        colors=['r', 'g', 'b', 'r', 'r', 'g', 'g', 'b', 'b'])

    def reset(self):
        self.origin = np.array([[0, 0, 0]])
        self.orientation = np.array([0, 0, 0])
        self.coordinate_system = self.scale * np.array([[1, 0, 0],
                                                        [0, 1, 0],
                                                        [0, 0, 1]])


def init_axis():
    ax = plt.figure().add_subplot(projection='3d')
    ax.set_xlim([-1, 1])
    ax.set_ylim([-1, 1])
    ax.set_zlim([0, 1])
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    plt.tight_layout()

    return ax


def main():
    ax = init_axis()
    plt.axis('on')
    axis_1 = CoordinateAxis(scale=1)
    axis_1.setup_plot(ax)

    axis_2 = CoordinateAxis(scale=1)
    axis_2.setup_plot(ax)

    theta = 0

    while True:
        rot = axis_2.rot_mat_axis(theta, [0, -np.sqrt(2) / 2, np.sqrt(2) / 2])
        axis_2.transform_frame(axis_2.homogenous_transformation_matrix(rot, np.array([[0, 0, 0]])))

        axis_2.update_plot()
        axis_2.reset()
        plt.pause(0.01)

        theta += 1

    plt.show()


if __name__ == '__main__':
    main()
