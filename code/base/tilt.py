import numpy as np

class Tilt:
    def __init__(self, n, period):
        self._period = period
        self._omega = 2 * np.pi / period
        self._q = np.zeros(n)
        self._v = np.zeros(n)
        self._a = np.zeros(n)

    def q(self, t):
        phi = self._phi(t)

        self._q[13] = 0.6 * (1 - np.cos(phi))

        return self._q

    def v(self, t):
        phi = self._phi(t)

        self._v[13] = 0.6 * self._omega * np.sin(phi)

        return self._v

    def a(self, t):
        phi = self._phi(t)

        self._a[13] = 0.6 * self._omega ** 2 * np.cos(phi)

        return self._a

    def _phi(self, t):
        return self._omega * np.fmod(t, self._period)
