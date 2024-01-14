# Author: ivantregear
# Date Created: 30/03/2023
# Description: 

# When I wrote this code only God I and knew how it worked.
# Now only god knows it.

import sympy as smp


class RobotDerivations:

    def __init__(self, dh, joints):
        self.dh_table = dh
        self.transformation_matrices = []
        self.rotation_matrices = []
        self.translation_matrices = []
        self.t_0_e = smp.zeros(4, 4)
        self.r_0_e = smp.zeros(3, 3)
        self.p_0_e = smp.zeros(3)
        self.joint_types = joints
        self.n_joints = len(joints)

        if len(joints) != self.dh_table.shape[0]:
            raise Exception("Warning: Number of joints not equal to number of rows in DH Table")

        self.vel = []
        self.omega = []
        self.j_e = 0
        self.j_0 = 0
        self.rot_jac = smp.zeros(6, 6)

        self.omega_dot = []
        self.vel_dot = []

        self.com_vel_dot = []
        self.com_symbols = None
        self.com_p = []

        self.com_force = []
        self.com_moment = []

        self.joint_force = []
        self.joint_moment = []

        smp.init_printing(wrap_line=False, use_latex=True)

    def dh_mat(self, row):
        a = row[0]
        alpha = row[1]
        d = row[2]
        theta = row[3]

        s = smp.sin
        c = smp.cos

        mat = smp.Matrix([[c(theta), -s(theta), 0, a],
                          [s(theta) * c(alpha), c(theta) * c(alpha), -s(alpha), -s(alpha) * d],
                          [s(theta) * s(alpha), c(theta) * s(alpha), c(alpha), c(alpha) * d],
                          [0, 0, 0, 1]])

        return mat

    def forward_kinematics(self, print_working=False):

        if print_working:
            print("\nIntermediate Matrices: \n")

        t_0_i = 1
        for idx in range(0, self.dh_table.shape[0]):
            mat_idx = self.dh_mat(self.dh_table.row(idx))
            t_0_i = t_0_i * mat_idx
            self.transformation_matrices += [mat_idx]

            if print_working:
                print("{}_{}_T".format(0, idx + 1))
                smp.pprint(smp.simplify(t_0_i))

        self.t_0_e = smp.simplify(t_0_i)

        for transform in self.transformation_matrices:
            self.rotation_matrices += [transform[:-1, :-1]]
            self.translation_matrices += [transform[:-1, -1]]

        self.p_0_e = self.t_0_e[:-1, -1]
        self.r_0_e = self.t_0_e[:-1, :-1]

    def derive_jacobian(self):

        self.omega += [smp.Matrix([[0], [0], [0]])]
        self.vel += [smp.Matrix([[0], [0], [0]])]

        t_dots = smp.symbols('thetadotdot1:7')
        d_dots = smp.symbols('ddotdot1:7')

        variables = []

        for idx in range(0, self.n_joints):

            if self.joint_types[idx] == 0:  # Prismatic joint

                variables += [d_dots[idx]]

                self.omega += [self.rotation_matrices[idx].T * self.omega[idx]]
                self.vel += [self.rotation_matrices[idx].T * (self.vel[idx] +
                                                              self.omega[idx].cross(self.translation_matrices[idx])) +
                             smp.Matrix([[0], [0], [d_dots[idx]]])]

            if self.joint_types[idx] == 1:  # Revolute joint

                variables += [t_dots[idx]]
                self.omega += [self.rotation_matrices[idx].T * self.omega[idx] + smp.Matrix([[0], [0], [t_dots[idx]]])]
                self.vel += [self.rotation_matrices[idx].T * (self.vel[idx] +
                                                              self.omega[idx].cross(self.translation_matrices[idx]))]

            if self.joint_types[idx] == 2:  # Fixed joint
                self.omega += [self.rotation_matrices[idx].T * self.omega[idx]]
                self.vel += [self.rotation_matrices[idx].T * (self.vel[idx] +
                                                              self.omega[idx].cross(self.translation_matrices[idx]))]

        eqs = [self.vel[-1][0, :][0], self.vel[-1][1, :][0], self.vel[-1][-1, :][0],
               self.omega[-1][0, :][0], self.omega[-1][1, :][0], self.omega[-1][-1, :][0]]

        j_e, _ = smp.linear_eq_to_matrix(eqs, variables)
        self.j_e = smp.simplify(j_e)

        self.rot_jac = smp.simplify(smp.Matrix(smp.BlockMatrix([[self.r_0_e, smp.zeros(3, 3)],
                                                                [smp.zeros(3, 3), self.r_0_e]])))

        self.j_0 = smp.simplify(self.rot_jac * self.j_e)

    def outward_iteration(self, gravity, vertical):

        g = smp.symbols('g')

        if gravity:
            if vertical == 'x':
                self.vel_dot += [smp.Matrix([[g], [0], [0]])]
            if vertical == 'y':
                self.vel_dot += [smp.Matrix([[0], [g], [0]])]
            if vertical == 'z':
                self.vel_dot += [smp.Matrix([[0], [0], [g]])]
        else:
            self.vel_dot += [smp.Matrix([[0], [0], [0]])]

        self.omega_dot += [smp.Matrix([[0], [0], [0]])]

        t_ddots = smp.symbols('thetaddotddot1:7')
        d_ddots = smp.symbols('dddotddot1:7')
        t_dots = smp.symbols('thetadotdot1:7')
        d_dots = smp.symbols('ddotdot1:7')

        for idx in range(0, self.n_joints):
            if self.joint_types[idx] == 0:  # Prismatic joint
                self.omega_dot += [self.rotation_matrices[idx].T * self.omega_dot[idx]]
                self.vel_dot += [self.rotation_matrices[idx].T * (self.vel_dot[idx] +
                                                                  self.omega_dot[idx].cross(
                                                                      self.translation_matrices[idx]) +
                                                                  self.omega[idx].cross(self.omega[idx].cross(
                                                                      self.translation_matrices[idx])))
                                 + 2 * self.omega[idx + 1].cross(smp.Matrix([[0], [0], [d_dots[idx]]])) +
                                 smp.Matrix([[0], [0], [d_ddots[idx]]])]

            if self.joint_types[idx] == 1:  # Revolute joint
                self.omega_dot += [self.rotation_matrices[idx].T * self.omega_dot[idx] +
                                   (self.rotation_matrices[idx].T * self.omega[idx]).cross(
                                       smp.Matrix([[0], [0], [t_dots[idx]]]))
                                   + smp.Matrix([[0], [0], [t_ddots[idx]]])]
                self.vel_dot += [self.rotation_matrices[idx].T * (self.vel_dot[idx] +
                                                                  self.omega_dot[idx].cross(
                                                                      self.translation_matrices[idx]) +
                                                                  self.omega[idx].cross(self.omega[idx].cross(
                                                                      self.translation_matrices[idx])))]

            if self.joint_types[idx] == 2:  # Fixed joint
                self.omega_dot += [self.rotation_matrices[idx].T * self.omega_dot[idx]]
                self.vel_dot += [self.rotation_matrices[idx].T * (self.vel_dot[idx] +
                                                                  self.omega_dot[idx].cross(
                                                                      self.translation_matrices[idx]) +
                                                                  self.omega[idx].cross(self.omega[idx].cross(
                                                                      self.translation_matrices[idx])))]

        masses = smp.symbols('m1:7')
        i_xx = smp.symbols('I_xx1:7')
        i_yy = smp.symbols('I_yy1:7')
        i_zz = smp.symbols('I_zz1:7')

        for idx in range(1, self.n_joints):

            signs = []

            for element in self.translation_matrices[idx]:
                if element == 0:
                    signs += [0]
                else:
                    signs += [smp.posify(smp.sign(element))[0]]

            self.com_p += [self.com_symbols[idx - 1] * smp.Matrix(signs)]

            self.com_vel_dot += [self.vel_dot[idx] +
                                 self.omega_dot[idx].cross(self.com_p[-1]) +
                                 self.omega[idx].cross(self.omega[idx].cross(self.com_p[-1]))]

            self.com_force += [masses[idx-1] * self.com_vel_dot[idx - 1]]
            self.com_moment += [smp.Matrix([[i_xx[idx-1], 0, 0],
                                            [0, i_yy[idx-1], 0],
                                            [0, 0, i_zz[idx-1]]]) * self.omega_dot[idx] +
                                self.omega_dot[idx].cross(smp.Matrix([[i_xx[idx-1], 0, 0],
                                                                          [0, i_yy[idx-1], 0],
                                                                          [0, 0, i_zz[idx-1]]]) * self.omega[idx])]

    def inward_iteration(self, f_e, m_e):

        self.joint_force += [smp.Matrix(f_e)]
        self.joint_moment += [smp.Matrix(m_e)]

        for idx in range(self.n_joints - 1, 0, -1):
            self.joint_force += [self.rotation_matrices[idx] * self.joint_force[-1] + self.com_force[idx-1]]
            self.joint_moment += [self.com_moment[idx-1] + self.rotation_matrices[idx]*self.joint_moment[-1] +
                                  self.com_p[idx-1].cross(self.com_force[idx-1]) +
                                  self.translation_matrices[idx].cross(self.rotation_matrices[idx] * self.joint_force[-1])]

        self.joint_force.reverse()
        self.joint_moment.reverse()

    def print_forward_kinematics(self):

        c1 = smp.Function('c')
        s1 = smp.Function('s')

        print("\nIndividual Transformation Matrices: \n")
        for i in range(len(self.transformation_matrices)):
            print("{}_{}_T:".format(i, i + 1))
            smp.pprint(self.transformation_matrices[i].replace(smp.sin, s1).replace(smp.cos, c1))

        print("\nOverall Transformation Matrix: \n")
        smp.pprint(self.t_0_e.replace(smp.sin, s1).replace(smp.cos, c1))

    def print_velocities(self):

        c1 = smp.Function('c')
        s1 = smp.Function('s')

        print("\nIndividual Joint Velocities:\n")
        for i in range(len(self.omega)):
            print("{}_{}_\u03A9:".format(i, i))
            smp.pprint(smp.simplify(self.omega[i].replace(smp.sin, s1).replace(smp.cos, c1)))
            print("{}_{}_V:".format(i, i))
            smp.pprint(smp.simplify(self.vel[i].replace(smp.sin, s1).replace(smp.cos, c1)))
            print()

    def print_accelerations(self):
        c1 = smp.Function('c')
        s1 = smp.Function('s')

        print("\nIndividual Joint Accelerations:\n")
        for i in range(len(self.omega_dot)):
            print("{}_{}_\u03A9\u0307:".format(i, i))
            smp.pprint(smp.simplify(self.omega_dot[i].replace(smp.sin, s1).replace(smp.cos, c1)))
            print("{}_{}_V\u0307:".format(i, i))
            smp.pprint(smp.simplify(self.vel_dot[i].replace(smp.sin, s1).replace(smp.cos, c1)))
            print()
        print("\nCentre of Mass Velocities:\n")
        for i in range(len(self.com_vel_dot)):
            print("{}_{}_V\u0307_c:".format(i + 1, i + 1))
            smp.pprint(smp.simplify(self.com_vel_dot[i].replace(smp.sin, s1).replace(smp.cos, c1)))

    def print_jacobian(self):
        c1 = smp.Function('c')
        s1 = smp.Function('s')

        print("\nJacobian in End-Effector Frame:")
        smp.pprint(self.j_e.replace(smp.sin, s1).replace(smp.cos, c1))

        print("\nJacobian Rotation Matrix:")
        smp.pprint(self.rot_jac.replace(smp.sin, s1).replace(smp.cos, c1))

        print("\nJacobian in Base Frame:")
        smp.pprint(self.j_0.replace(smp.sin, s1).replace(smp.cos, c1))
        print()

    def print_forces(self):
        c1 = smp.Function('c')
        s1 = smp.Function('s')

        print("\nForces Acting on COM:\n")
        for i in range(len(self.com_force)):
            print("{}_{}_F:".format(i, i))
            smp.pprint(smp.simplify(self.com_force[i].replace(smp.sin, s1).replace(smp.cos, c1)))
            print("{}_{}_M:".format(i, i))
            smp.pprint(smp.simplify(self.com_moment[i].replace(smp.sin, s1).replace(smp.cos, c1)))
            print()

        print("\nForces Acting on Joints:\n")
        for i in range(len(self.joint_force)):
            print("{}_{}_F:".format(i, i))
            smp.pprint(smp.simplify(self.joint_force[i].replace(smp.sin, s1).replace(smp.cos, c1)))
            print("{}_{}_M:".format(i, i))
            smp.pprint(smp.simplify(self.joint_moment[i].replace(smp.sin, s1).replace(smp.cos, c1)))
            print()

