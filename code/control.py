from base import Talos, Tilt

import cvxopt as opt
import numpy as np
import pinocchio as pin

class ControlledTalos(Talos):
    def __init__(self):
        super(ControlledTalos, self).__init__()

        self.trajectory = Tilt(self, 10.0)

        self.support = [
            self.get_frame_index("leg_left_sole_fix_joint"),
            self.get_frame_index("leg_right_sole_fix_joint")
        ]

        self.k_p = 50.0 # proportional gain
        self.k_d = 2.0 * np.sqrt(self.k_p) # derivative gain

        self.W = np.eye(self.model.nv) # tracking weights

        self.G = np.zeros((self.model.nv, self.model.nv))
        for i in range(0, 6):
            self.G[i][i] = 5.0 # floaing base torques weights

        self.k_r = 1e-3 # regularization coefficient

    def get_support_jacobian(self):
        return np.vstack([self.get_frame_jacobian(support) for support in self.support])

    def get_support_jacobian_variation(self):
        return np.vstack([self.get_frame_jacobian_variation(support) for support in self.support])

    def update(self, t, dt):
        ### Compute terms required to formulate optimization problem ###

        H = self.compute_joint_space_inertia()
        c = self.compute_non_linear_term()

        self.compute_joint_jacobians()
        self.update_frame_placements()

        J = self.get_jacobian()

        J_sup = self.get_support_jacobian()
        dot_J_sup = self.get_support_jacobian_variation()

        q_traj = self.trajectory.q(t)
        v_traj = self.trajectory.v(t)
        a_traj = self.trajectory.a(t)

        a_des = - self.k_p * self.projection @ (self.q - q_traj) - self.k_d * (self.v - v_traj) + a_traj

        nv = self.model.nv # dimension of both acceleration and torques
        nf = self.model.njoints * 6 # dimension of forces
        nc = J_sup.shape[0]

        ### Formulate objective ###

        P = opt.matrix(np.block([
            [self.W,             np.zeros((nv, nv)), np.zeros((nv, nf))],
            [np.zeros((nv, nv)), self.G,             np.zeros((nv, nf))],
            [np.zeros((nf, nv)), np.zeros((nf, nv)), np.zeros((nf, nf))]
        ]))

        q = opt.matrix(np.hstack([
            -1.0 * a_des.T @ self.W,
            np.zeros(nv),
            np.zeros(nf),
        ]))

        ### Formulate L2 regularization ###

        R = opt.matrix(np.eye(nv + nv + nf) * self.k_r)

        ### Formulate equality constraints ###

        A = opt.matrix(np.block([
            [H,     -1.0 * np.eye(nv),   -1.0 * J.T        ],
            [J_sup, np.zeros((nc, nv)),  np.zeros((nc, nf))]
        ]))

        b = opt.matrix(np.hstack([
            -1.0 * c,
            -1.0 * dot_J_sup @ self.v,
        ]))

        ### Solve ###

        solution = opt.solvers.qp(P + R, q, A = A, b = b)
        x = np.array(solution['x'])
        x = x.reshape(x.shape[0])

        ### Apply forward dynamics ###

        u = x[nv : nv + nv]
        f = x[nv + nv : nv + nv + nf]
        self.apply_forward_dynamics(u, f, dt)

def main():
    talos = ControlledTalos()
    time = 0.0
    delta_time = 1e-3
    iteration = 0
    while True:
        time += delta_time
        iteration += 1
        talos.update(time, delta_time)
        if iteration % 20 == 0:
            talos.display()

if __name__ == '__main__':
    main()
