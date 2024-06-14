from base import Talos, Tilt

import cvxopt as opt
import numpy as np
import pinocchio as pin
import time

class ControlledTalos(Talos):
    def __init__(self):
        super(ControlledTalos, self).__init__()

        ### Set trajectory to follow ###

        self.trajectory = Tilt(self, 10.0)

        ### Set support ###

        self.support = [
            self.get_frame_index("leg_left_sole_fix_joint"),
            self.get_frame_index("leg_right_sole_fix_joint")
        ]

        ### set PD gains ###

        self.k_l = 1000.0
        self.d_l = 2.0 * np.sqrt(self.k_l)

        self.k_a = 1000.0
        self.d_a = 2.0 * np.sqrt(self.k_l)

        self.k_t = 50.0
        self.d_t = 2.0 * np.sqrt(self.k_t)

        ### set objective weight ###

        self.w_l = 10.0
        self.w_a = 1.0

        self.W = np.eye(self.model.nv)

        self.G = np.zeros((self.model.nv, self.model.nv))
        for i in range(0, 6):
            self.G[i][i] = 5.0

        ### set regularization ###

        self.k_r = 1e-3

    def get_support_jacobian(self):
        return np.vstack([self.get_frame_jacobian(support) for support in self.support])

    def get_support_jacobian_variation(self):
        return np.vstack([self.get_frame_jacobian_variation(support) for support in self.support])

    def update(self, t, dt):
        ### Compute all terms required to formulate optimization problem ###

        H = self.compute_joint_space_inertia()
        c = self.compute_non_linear_term()

        self.compute_joint_jacobians()
        self.update_frame_placements()

        J = self.get_jacobian()

        J_sup = self.get_support_jacobian()
        dot_J_sup = self.get_support_jacobian_variation()

        A = self.compute_centroidal_momentum_matrix()
        [A_l, A_a] = np.vsplit(A, 2)

        dot_A = self.compute_centroidal_momentum_matrix_variation()
        [dot_A_l, dot_A_a] = np.vsplit(dot_A, 2)

        com, dot_com = self.compute_center_of_mass()

        dot_P_des = self.m * (- self.k_l * (com - self.com_ref) - self.d_l * dot_com)

        q_traj = self.trajectory.q(t)
        v_traj = self.trajectory.v(t)
        a_traj = self.trajectory.a(t)

        a_des = - self.k_t * self.projection @ (self.q - q_traj) - self.d_t * (self.v - v_traj) + a_traj

        nv = self.model.nv # dimension of both acceleration and torques
        nf = self.model.njoints * 6 # dimension of forces
        nc = J_sup.shape[0]

        ### Formulate objective ###

        # print((dot_P_des - (A_l @ self.a + dot_A_l @ self.v)))
        # print((dot_P_des - dot_A_l @ self.v))

        # print(com - self.com_ref)

        # print(dot_P_des - self.m * self.g)

        Q = opt.matrix(np.block([
            [self.w_l * A_l.T @ A_l + self.W, np.zeros((nv, nv)), np.zeros((nv, nf))],
            [np.zeros((nv, nv)),              self.G,             np.zeros((nv, nf))],
            [np.zeros((nf, nv)),              np.zeros((nf, nv)), np.zeros((nf, nf))]
        ]))

        p = opt.matrix(np.hstack([
            -1.0 * (self.w_l * (dot_P_des - dot_A_l @ self.v).T @ A_l + a_des.T @ self.W),
            np.zeros(nv),
            np.zeros(nf),
        ]))

        ### Formulate L2 regularization ###

        R = opt.matrix(np.eye(nv + nv + nf) * self.k_r)

        ### Formulate equality constraints ###

        B = opt.matrix(np.block([
            [H,     -1.0 * np.eye(nv),   -1.0 * J.T        ],
            [J_sup, np.zeros((nc, nv)),  np.zeros((nc, nf))]
        ]))

        d = opt.matrix(np.hstack([
            -1.0 * c,
            -1.0 * dot_J_sup @ self.v,
        ]))

        ### Solve ###

        solution = opt.solvers.qp(Q + R, p, A = B, b = d)
        x = np.array(solution['x'])
        x = x.reshape(x.shape[0])

        ### Apply forward dynamics ###

        u = x[nv : nv + nv]
        f = x[nv + nv : nv + nv + nf]
        self.apply_forward_dynamics(u, f, dt)

def main():
    talos = ControlledTalos()

    simulation_time = 0.0
    simulation_delta_time = 0.008 # 120 hz
    while True:
        initial_time = time.time()
        simulation_time += simulation_delta_time
        talos.update(simulation_time, simulation_delta_time)
        talos.display()
        update_duration = time.time() - initial_time
        if update_duration < simulation_delta_time:
            time.sleep(simulation_delta_time - update_duration)

if __name__ == '__main__':
    main()
