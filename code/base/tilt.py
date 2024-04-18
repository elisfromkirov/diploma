import numpy as np

class Tilt:
    def __init__(self, robot, period):
        self._period = period

        self._omega = 2 * np.pi / period

        self._q = np.zeros(robot.model.nq)
        self._v = np.zeros(robot.model.nv)
        self._a = np.zeros(robot.model.nv)

        torso = robot.model.joints[robot.model.getJointId("torso_2_joint")]

        self._torso_q_index = torso.idx_q
        self._torso_v_index = torso.idx_v
        self._torso_a_index = torso.idx_v

        l_shoulder = robot.model.joints[robot.model.getJointId("arm_left_1_joint")]

        self._l_shoulder_q_index = l_shoulder.idx_q
        self._l_shoulder_v_index = l_shoulder.idx_v
        self._l_shoulder_a_index = l_shoulder.idx_v

        l_arm = robot.model.joints[robot.model.getJointId("arm_left_2_joint")]

        self._l_arm_q_index = l_arm.idx_q
        self._l_arm_v_index = l_arm.idx_v
        self._l_arm_a_index = l_arm.idx_v

        r_shoulder = robot.model.joints[robot.model.getJointId("arm_right_1_joint")]

        self._r_shoulder_q_index = r_shoulder.idx_q
        self._r_shoulder_v_index = r_shoulder.idx_v
        self._r_shoulder_a_index = r_shoulder.idx_v

        r_arm = robot.model.joints[robot.model.getJointId("arm_right_2_joint")]

        self._r_arm_q_index = r_arm.idx_q
        self._r_arm_v_index = r_arm.idx_v
        self._r_arm_a_index = r_arm.idx_v

    def q(self, t):
        phi = (1 - np.cos(self._phi(t)))

        self._q[self._torso_q_index]      =  0.6 * phi
        self._q[self._l_shoulder_q_index] = -0.8 * phi
        self._q[self._l_arm_q_index]      =  0.6 * phi
        self._q[self._r_shoulder_q_index] =  0.8 * phi
        self._q[self._r_arm_q_index]      = -0.6 * phi

        return self._q

    def v(self, t):
        phi = self._omega * np.sin(self._phi(t))

        self._v[self._torso_v_index]      =  0.6 * phi
        self._v[self._l_shoulder_v_index] = -0.8 * phi
        self._v[self._l_arm_v_index]      =  0.6 * phi
        self._v[self._r_shoulder_v_index] =  0.8 * phi
        self._v[self._r_arm_v_index]      = -0.6 * phi

        return self._v

    def a(self, t):
        phi = self._omega ** 2 * np.cos(self._phi(t))

        self._a[self._torso_a_index]      =  0.6 * phi
        self._a[self._l_shoulder_a_index] = -0.8 * phi
        self._a[self._l_arm_a_index]      =  0.6 * phi
        self._a[self._r_shoulder_a_index] =  0.8 * phi
        self._a[self._r_arm_a_index]      = -0.6 * phi

        return self._a

    def _phi(self, t):
        return self._omega * np.fmod(t, self._period)
