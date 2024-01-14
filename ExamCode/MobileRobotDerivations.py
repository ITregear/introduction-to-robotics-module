# Author: ivantregear
# Date Created: 17/04/2023
# Description: 

# When I wrote this code only God I and knew how it worked.
# Now only god knows it.


import sympy as smp
import numpy as np


class Wheel:
    def __init__(self, alpha, beta, l, r):
        self.alpha = alpha
        self.beta = beta
        self.l = l
        self.r = r

    def rolling_condition(self):

        s = smp.sin
        c = smp.cos

        mat = smp.Matrix([s(self.alpha+self.beta), -c(self.alpha+self.beta), -self.l*c(self.beta)])

        return mat

    def sliding_condition(self):

        s = smp.sin
        c = smp.cos

        mat = smp.Matrix([c(self.alpha+self.beta), s(self.alpha+self.beta), self.l*s(self.beta)])

        return mat


class MobileRobot:
    def __init__(self):
        self.wheels = []

        self.condition_matrix = None
        self.radii = None
        self.phis = []
        self.phi_array = None

        self.actuated = []
        self.condition_mask = []
        self.fk_mask = []
        self.fk_indexes = None
        self.ik_mask = []
        self.ik_indexes = None

        self.fk_radii = None
        self.fk_matrix = None
        self.fk_phi = None
        self.fk_matrix_inv = None
        self.fk = None

        self.ik_radii = None
        self.ik_matrix = None
        self.ik_phi = None
        self.ik = None

        self.rolling_conditions = None
        self.sliding_conditions = None

        self.n_wheels = 0

        s = smp.sin
        c = smp.cos
        x, y, theta = smp.symbols('x, y, theta')
        ixdot, iydot, thetadot = smp.symbols('xdot_i, ydot_i, thetadot')

        self.R_0_I = smp.Matrix([[c(theta), s(theta), 0],
                                 [-s(theta), c(theta), 0],
                                 [0, 0, 1]])
        self.R_I_0 = self.R_0_I.T

        self.I_P = smp.Matrix([ixdot, iydot, thetadot])

    def add_wheel(self, a, b, l, r, actuated, phi):
        if actuated:
            self.actuated += [True]
        else:
            self.actuated += [False]
        self.phis += [phi]
        self.wheels += [Wheel(a, b, l, r)]

    def get_all_conditions(self):

        # Populates rolling conditions, then sliding conditions per wheel

        self.n_wheels = len(self.wheels)

        self.condition_matrix = smp.zeros(2*self.n_wheels, 3)

        self.radii = smp.zeros(self.n_wheels, self.n_wheels)
        self.fk_radii = smp.zeros(self.n_wheels * 2, self.n_wheels * 2)  # Size [2*n, 2*n]
        self.fk_phi = smp.zeros(self.n_wheels * 2, 1)

        self.condition_mask = [0] * 2*self.n_wheels

        # Rolling Condition = 0
        # Sliding Condition = 1

        for i in range(self.n_wheels):
            self.condition_matrix[i, :] = self.wheels[i].rolling_condition().T
            self.condition_matrix[self.n_wheels + i, :] = self.wheels[i].sliding_condition().T
            self.condition_mask[i] = False
            self.condition_mask[self.n_wheels + i] = True
            self.radii[i, i] = self.wheels[i].r
            self.fk_radii[i, i] = self.wheels[i].r
            self.fk_radii[self.n_wheels + i, 0] = 0  # Unnecessary, but demonstrates intention
            self.fk_phi[i] = self.phis[i]
            self.fk_phi[self.n_wheels + i] = 0  # Unnecessary, but demonstrates intention

        self.rolling_conditions = self.condition_matrix[:self.n_wheels, :]
        self.sliding_conditions = self.condition_matrix[self.n_wheels:, :]

        self.phi_array = smp.Matrix(self.phis)

        return self.condition_matrix

    def forward_kinematics(self):

        self.fk_mask = [a or b for a, b in zip(self.condition_mask, self.actuated*2)]
        self.fk_indexes = np.argwhere(np.array(self.fk_mask)).T[0]

        smp.pprint(self.condition_matrix)

        self.fk_matrix = self.condition_matrix.copy()
        self.ik_matrix = self.condition_matrix.copy()
        self.ik_radii = self.fk_radii.copy()
        self.ik_phi = self.fk_phi.copy()

        # Removing False element indicies from fk_mask
        self.fk_matrix = self.fk_matrix[self.fk_indexes.tolist(), :]
        self.fk_radii = self.fk_radii[self.fk_indexes.tolist(), self.fk_indexes.tolist()]
        self.fk_phi = self.fk_phi[self.fk_indexes.tolist(), :]

        # Now removing duplicated rows, so only independent equations remain

        del_count = 0

        for i in range(0, self.fk_matrix.shape[0] - 1):
            if self.fk_matrix[i - del_count, :] == self.fk_matrix[i + 1 - del_count, :]:
                self.fk_matrix.row_del(i+1-del_count)
                self.fk_radii.row_del(i-del_count)
                self.fk_radii.col_del(i-del_count)
                self.fk_phi.row_del(i-del_count)
                del_count += 1

        self.fk_matrix_inv = smp.simplify(self.fk_matrix.inv())

        self.fk = self.R_I_0 @ self.fk_matrix_inv @ self.fk_radii @ self.fk_phi

    def inverse_kinematics(self):

        self.ik_mask = [not a for a in self.fk_mask]
        self.ik_indexes = np.argwhere(np.array(self.ik_mask)).T[0]

        self.ik_radii = self.ik_radii[self.ik_indexes.tolist(), self.ik_indexes.tolist()]
        self.ik_matrix = self.ik_matrix[self.ik_indexes.tolist(), :]
        self.ik_phi = self.ik_phi[self.ik_indexes.tolist(), :]

        self.ik = self.radii.inv() @ self.rolling_conditions @ self.R_0_I @ self.I_P

        smp.pprint(smp.simplify(self.ik))

    def print_conditions(self):
        print("Rolling Conditions:")
        smp.pprint(self.rolling_conditions)
        smp.pprint(self.radii)
        print("\nSliding Conditions:")
        smp.pprint(self.sliding_conditions)

    def print_forward_kinematics(self):
        print("\nForward Kinematics:")
        smp.pprint(self.fk)

        print("\nPassive Wheel Angular Velocities:")
        smp.pprint(self.passive_ik)

    def manoeuvrability(self):
        c1 = self.condition_matrix[self.n_wheels:, :].copy()

        c1_rank = smp.Matrix.rank(c1)

        del_count = 0

        for i in range(self.n_wheels):
            if not self.actuated[i]:
                c1.row_del(i - del_count)
                del_count += 1

        c1s_rank = smp.Matrix.rank(c1)

        print("Degree of Mobility:", 3-c1_rank)
        print("Degree of Steerability:", c1s_rank)
        print("Degree of Manoeuvrability:", (3-c1_rank)+c1s_rank)


def tricycle():
    robot = MobileRobot()
    alpha, beta, L, d, R, r = smp.symbols('alpha, beta, L, d, R, r')
    phis = smp.symbols('phidot1:5')
    pi = smp.pi

    # alpha, beta, L, R, Acutuated

    robot.add_wheel(pi / 2, 0, d, r, False, phis[0])
    robot.add_wheel(-pi / 2, pi, d, r, False, phis[1])
    robot.add_wheel(0, beta, L, R, True, phis[2])

    robot.get_all_conditions()
    robot.forward_kinematics()
    robot.inverse_kinematics()


def bicycle():
    robot = MobileRobot()
    alpha, beta, L, d, R, r = smp.symbols('alpha, beta, L, d, R, r')
    phis = smp.symbols('phidot1:5')
    pi = smp.pi

    # alpha, beta, L, R, Acutuated, phi

    robot.add_wheel(pi, -pi/2, L/2, R, False, phis[0])
    robot.add_wheel(0, beta, L/2, R, True, phis[1])

    robot.get_all_conditions()
    robot.forward_kinematics()
    robot.inverse_kinematics()


def differential_drive():
    robot = MobileRobot()
    alpha, beta, L, d, R, r = smp.symbols('alpha, beta, L, d, R, r')
    phis = smp.symbols('phidot1:5')
    pi = smp.pi

    # alpha, beta, L, R, Acutuated, phi

    robot.add_wheel(pi / 2, 0, L, r, True, smp.symbols("\dot{\phi}_{left}"))
    robot.add_wheel(-pi / 2, pi, L, r, True, smp.symbols("\dot{\phi}_{right}"))

    robot.get_all_conditions()
    robot.forward_kinematics()
    robot.inverse_kinematics()


def omni_wheel():
    robot = MobileRobot()
    alpha, beta, L, d, R, r = smp.symbols('alpha, beta, L, d, R, r')
    phis = smp.symbols('phidot1:5')
    pi = smp.pi

    # alpha, beta, L, R, Acutuated, phi

    robot.add_wheel(pi / 3, 0, L, r, True, smp.symbols("\dot{\phi}_{1}"))
    robot.add_wheel(pi, 0, L, r, True, smp.symbols("\dot{\phi}_{2}"))
    robot.add_wheel(-pi / 3, 0, L, r, True, smp.symbols("\dot{\phi}_{3}"))

    robot.get_all_conditions()
    robot.forward_kinematics()
    robot.inverse_kinematics()


if __name__ == "__main__":
    # tricycle()
    # bicycle()
    differential_drive()
    # omni_wheel()
