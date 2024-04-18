from base import Talos, Tilt

class AnimatedTalos(Talos):
    def __init__(self):
        super(AnimatedTalos, self).__init__()

        self.animation = Tilt(self, 10.0)

    def update(self, t, dt):
        self.q = self.animation.q(t)

def main():
    talos = AnimatedTalos()
    time = 0.0
    delta_time = 1e-4
    iteration = 0
    while True:
        time += delta_time
        iteration += 1
        talos.update(time, delta_time)
        if iteration % 20 == 0:
            talos.display()

if __name__ == '__main__':
    main()
