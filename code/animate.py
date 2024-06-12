from base import Talos, Tilt

import time

class AnimatedTalos(Talos):
    def __init__(self):
        super(AnimatedTalos, self).__init__()

        self.animation = Tilt(self, 10.0)

    def update(self, t, dt):
        self.q = self.animation.q(t)

def main():
    talos = AnimatedTalos()

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
