from base import Talos, Tilt, loop

import cvxopt as opt
import numpy as np
import pinocchio as pin

def debug_print(A):
    with np.printoptions(threshold = np.inf, linewidth = 10000):
        print(A)
    print('')

class ControlledTalos(Talos):
    def __init__(self):
        super(ControlledTalos, self).__init__()

        self.support = [
            self.model.getFrameId("leg_left_sole_fix_joint"),
            self.model.getFrameId("leg_right_sole_fix_joint")
        ]

        self.k_p = 50.0
        self.k_d = 2.0 * np.sqrt(self.k_p)

        self.trajectory = Tilt(10.0) # fix this a little, replace magic numbers with getJointId('lala-joint')

        self.W = np.eye(self.model.nv) # do smth with fucking model.nv

        self.G = np.zeros((self.model.nv, self.model.nv))
        for i in range(0, 6):
            self.G[i][i] = 1

    # THIS BLOCK OF FUNCTIONS SHOULD BE PUT TO BASE CLASS #

    def apply_forward_kinematics(self):
        pin.forwardKinematics(self.model, self.data, self.q)

    def crba(self):
        return pin.crba(self.model, self.data, self.q)

    def rnea(self, a = None):
        if a == None:
            return pin.rnea(self.model, self.data, self.q, self.v, self.zeros)
        pin.rnea(self.model, self.data, self.q, self.v, a)

    def compute_joint_space_inertia(self):
        return pin.crba(self.model, self.data, self.q)

    def compute_non_linear_effects(self):
        return pin.rnea(self.model, self.data, self.q, self.v, self.zeros)

    def compute_joint_jacobians(self):
        pin.computeJointJacobians(self.model, self.data)

    def update_frame_placements(self):
        pin.updateFramePlacements(self.model, self.data)

    def get_frame_jacobian(self, frame):
        return pin.getFrameJacobian(self.model, self.data, frame, pin.LOCAL)

    def get_frame_jacobian_variation(self, frame):
        return pin.getFrameJacobianTimeVariation(self.model, self.data, frame, pin.LOCAL)

    # THIS BLOCK OF FUNCTIONS SHOULD BE PUT TO BASE CLASS #

    def get_support_jacobian(self):
        return np.vstack([self.get_frame_jacobian(support) for support in self.support])

    def get_support_jacobian_variation(self):
        return np.vstack([self.get_frame_jacobian_variation(support) for support in self.support])

    # def update():

    def _update(self, t, dt):
        H = self.compute_joint_space_inertia()

        c = self.compute_non_linear_effects()

        self.apply_forward_kinematics()

        self.compute_joint_jacobians()

        self.update_frame_placements()

        J_sup = self.get_support_jacobian()
        dot_J_sup = self.get_support_jacobian_variation()

        q_traj = self.trajectory.q(t)
        v_traj = self.trajectory.v(t)
        a_traj = self.trajectory.a(t)

        a_des = - self.k_p * self.projection @ (self.q - q_traj) - self.k_d * (self.v - v_traj) + a_traj

        ### Just to make matrix more readalbe ###

        nv = self.model.nv
        nc = J_sup.shape[0]

        ### Formulate objective ###

        P = opt.matrix(np.block([
            [self.W,             np.zeros((nv, nv)), np.zeros((nv, nc))],
            [np.zeros((nv, nv)), self.G,             np.zeros((nv, nc))],
            [np.zeros((nc, nv)), np.zeros((nc, nv)), np.zeros((nc, nc))]
        ]))

        q = opt.matrix(np.hstack([
            a_des.T @ self.W,
            np.zeros(nv),
            np.zeros(nc),
        ]))

        ### Formulate L2 regularization ###

        R = opt.matrix(np.eye(nv + nv + nc) * 1e-3)

        ### Formulate equality constraints ###

        A = opt.matrix(np.block([
            [H,     -1.0 * np.eye(nv),   J_sup.T           ],
            [J_sup, np.zeros((nc, nv)),  np.zeros((nc, nc))]
        ]))

        b = opt.matrix(np.hstack([
            -1.0 * c,
            -1.0 * dot_J_sup @ self.v,
        ]))

        ### Solve ###

        solution = opt.solvers.qp(P + R, q, A = A, b = b)
        x = np.array(solution['x'])
        print(x[nv:nv+6])

        ### Apply forward dynamics ###

def main():
    talos = ControlledTalos()
    talos.update(0, 1e-4)
    # talos.display()
    # loop(talos)

if __name__ == '__main__':
    main()

# stuff from past #

# how to obtain required for animation quantities
# torso_pitch = self.model.getJointId("torso_2_joint")
# print(self.model.joints[torso_pitch].idx_q)
# print(self.model.joints[torso_pitch])

# jf = self.model.getFrameId("leg_right_6_joint")
# lf = self.model.getFrameId("leg_right_6_link")

# sjf = self.model.getFrameId("leg_right_sole_fix_joint")
# slf = self.model.getFrameId("right_sole_link")

# pin.framesForwardKinematics(self.model, self.data, self.q)

# print(self.data.oMf[jf])
# print(self.data.oMf[lf])
# print(self.data.oMf[sjf])
# print(self.data.oMf[slf])

# self.q[7 + 11] = 0.5

# pin.framesForwardKinematics(self.model, self.data, self.q)

# print(self.data.oMf[jf])
# print(self.data.oMf[lf])
# print(self.data.oMf[sjf])
# print(self.data.oMf[slf])

# let's call it support frame
# because it's a frame associated with support

# # updates joints positions according to given configuration
# pin.forwardKinematics(self.model, self.data, self.q)

# # updates frames positions according to data stored in data
# pin.framesForwardKinematics(self.model, self.data)

# getFrameId("name")

# pin.forwardKinematics()

# we want all terms to be zero

# I want to know linear velocity in world frame
# this is oMf.translate * fJ * dq_dt

# pass

# tau = - self.k_p * self.projection @ (self.q - q_des) - self.k_d * (self.v - v_des) + a_des

# tau = pin.rnea(self.model, self.data, self.q, self.v, self.zeros)

# nrf = np.zeros(3)
# if self.q[2] < 0.0:
#     nrf[2] = - self.f_p * self.q[2] - self.f_d * self.v[2] + 2.7 * self.g

# f = [pin.Force.Zero()]*self.model.njoints
# f[0].linear = nrf

# self.a = pin.aba(self.model, self.data, self.q, self.v, tau, f)

# self.a = pin.aba(self.model, self.data, self.q, self.v, tau)

# self.v += self.a * dt
# self.q = pin.integrate(self.model, self.q, self.v * dt)
