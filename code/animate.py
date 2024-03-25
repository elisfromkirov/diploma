from base import Talos, Tilt, loop

class AnimatedTalos(Talos):
    def __init__(self):
        super(AnimatedTalos, self).__init__()

        self.animation = Tilt(self.model.nv, 10.0)

    def update(self, t, dt):
        self.q = self.animation.q(t)

def main():
    talos = AnimatedTalos()
    loop(talos)

if __name__ == '__main__':
    main()
