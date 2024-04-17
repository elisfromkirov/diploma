import numpy as np

class Tilt:
    def __init__(self, period):
        self._period = period
        self._omega = 2 * np.pi / period
        self._q = np.zeros(39)
        self._v = np.zeros(38)
        self._a = np.zeros(38)

    def q(self, t):
        phi = self._phi(t)

        self._q[7 + 13] = 0.6 * (1 - np.cos(phi))

        return self._q

    def v(self, t):
        phi = self._phi(t)

        self._v[6 + 13] = 0.6 * self._omega * np.sin(phi)

        return self._v

    def a(self, t):
        phi = self._phi(t)

        self._a[6 + 13] = 0.6 * self._omega ** 2 * np.cos(phi)

        return self._a

    def _phi(self, t):
        return self._omega * np.fmod(t, self._period)
