import numpy as np

class Tilt:
    def __init__(self, robot, period):
        torso_pitch = robot.model.joints[robot.model.getJointId("torso_2_joint")]

        self._period = period
        self._omega = 2 * np.pi / period
        self._q = np.zeros(robot.model.nq)
        self._v = np.zeros(robot.model.nv)
        self._a = np.zeros(robot.model.nv)
        self._q_index = torso_pitch.idx_q
        self._v_index = torso_pitch.idx_v
        self._a_index = torso_pitch.idx_v

    def q(self, t):
        self._q[self._q_index] = 0.6 * (1 - np.cos(self._phi(t)))
        return self._q

    def v(self, t):
        self._v[self._v_index] = 0.6 * self._omega * np.sin(self._phi(t))
        return self._v

    def a(self, t):
        self._a[self._a_index] = 0.6 * self._omega ** 2 * np.cos(self._phi(t))
        return self._a

    def _phi(self, t):
        return self._omega * np.fmod(t, self._period)
