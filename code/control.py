from base import Talos, Tilt, loop

import cvxopt as cvx
import numpy as np
import pinocchio as pin

class ControlledTalos(Talos):
    def __init__(self):
        super(ControlledTalos, self).__init__()

        self.k_p = 50.0
        self.k_d = 2.0 * np.sqrt(self.k_p)

        self.W = np.eye(self.model.nv)

        self.support = [
            self.model.getFrameId("leg_left_sole_fix_joint"),
            self.model.getFrameId("leg_right_sole_fix_joint")
        ]

    def get_support_jacobian(self):
        return np.vstack([pin.getFrameJacobian(support) for support in self.support])

    def get_support_jacobian_variation(self):
        return np.vstack([pin.getFrameJacobianTimeVariation(support) for support in self.support])

    def update(self, t, dt):
        # at the begining of loop we have all required quantities

        # tracking objective is to minimize
        # || W (a - a_des) ||_2
        # where a_des is PD(q_traj, v_traj, a_traj)

        q_traj = self.animation.q(t)
        v_traj = self.animation.v(t)
        a_traj = self.animation.a(t)

        a_des = - self.k_p * self.projection @ (self.q - q_traj) - self.k_d * (self.v - v_traj) + a_traj

        J_sup = self.get_support_jacobian()
        dot_J_sup = self.get_support_jacobian_variation()

        #

        #

        # optimizer solves the following problem
        # minimize \frac{1}}{2}Px^T x + qx
        # s.t. Gx < h,
        #      Ax = b

        # P = self.W
        # q = a_des @ self.W

        # solver = cvx.solvers.qp(P, q)

        # u is numpy array
        # f is numpy array
        # we also need a jacobian

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

        pass

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

def main():
    talos = ControlledTalos()
    talos.update()
    talos.display()
    # loop(talos)

if __name__ == '__main__':
    main()
