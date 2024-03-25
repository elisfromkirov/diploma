from base import Talos, Tilt, loop

import numpy as np
import pinocchio as pin

class ControlledTalos(Talos):
    def __init__(self):
        super(ControlledTalos, self).__init__()

        self.zeros = np.zeros(self.model.nv)

        self.k_p = 50.0
        self.k_d = 2.0 * np.sqrt(self.k_p)
        self.k = 1.0

        self.animation = Tilt(self.model.nv, 10.0)

    def update(self, t, dt):
        q_des = self.animation.q(t)
        v_des = self.animation.v(t)
        a_des = self.animation.a(t)

        tau = - self.k_p * (self.q - q_des) - self.k_d * (self.v - v_des) + self.k * a_des

        self.a = pin.aba(self.model, self.data, self.q, self.v, tau)
        self.v += self.a * dt
        self.q = pin.integrate(self.model, self.q, self.v * dt)

def main():
    talos = ControlledTalos()
    loop(talos)

if __name__ == '__main__':
    main()
