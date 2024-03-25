import numpy as np
import os.path as fs
import pinocchio as pin
import pinocchio.visualize as vis

WORKSPACE_DIR = fs.dirname(fs.abspath('{}/../'.format(__file__)))

class Trajectory:
    def __init__(self, omega, amplitude):
        self.omega = omega
        self.amplitude = amplitude

    def position(self, t):
        return self.amplitude * np.sin(self.omega * t)

    def velocity(self, t):
        return self.omega * self.amplitude * np.cos(self.omega * t)

    def acceleration(self, t):
        return -1 * self.omega ** 2 * self.amplitude * np.sin(self.omega * t)

class Robot:
    def __init__(self, model_file_path):
        # robot model parameters
        self.model, self.collision_model, self.visual_model = pin.buildModelsFromUrdf(model_file_path, WORKSPACE_DIR)

        # data for computation
        self.data = self.model.createData()

        # robot vistualizer
        self.visualizer = vis.GepettoVisualizer(self.model, self.collision_model, self.visual_model)
        self.visualizer.initViewer()
        self.visualizer.loadViewerModel("pinocchio")

        # current robot configuration
        self.q = pin.neutral(self.model)
        self.vq = np.zeros(self.model.nv)
        self.aq = np.zeros(self.model.nv)

        # auxiliary values
        self.zeros = np.zeros(self.model.nv)

        # proportional-derivative controller parameters
        self.k_p = 50.0
        self.k_d = 2.0 * np.sqrt(self.k_p)
        self.k = 1.0

        # trajectory
        omega = np.ones(self.model.nv)
        amplitude = np.zeros(self.model.nv)
        amplitude[0] = 1.0
        # amplitude[1] = 1.0
        self.trajectory = Trajectory(omega, amplitude)

    def dysplay(self):
        self.visualizer.display(self.q)

    def crba(self):
        return pin.crba(self.model, self.data, self.q)

    def rnea(self, aq = None):
        if aq is None:
            aq = self.zeros
        return pin.rnea(self.model, self.data, self.q, self.vq, aq)

    def track_tracjectory_step(self, t, dt):
        H = self.crba()
        c = self.rnea()

        q_des = self.trajectory.position(t)
        vq_des = self.trajectory.velocity(t)
        aq_des = self.trajectory.acceleration(t)

        tau = - self.k_p * (self.q - q_des) - self.k_d * (self.vq - vq_des) + self.k * aq_des
        # print(type(tau))
        print(tau.shape[0])
        print(self.model.nq)

        self.aq = np.linalg.inv(H) @ (tau - c)
        self.vq += self.aq * dt
        self.q = pin.integrate(self.model, self.q, self.vq * dt)

def main():
    robot = Robot(f'{WORKSPACE_DIR}/example-robot-data/robots/ur_description/urdf/ur5_robot.urdf')
    robot.dysplay()

    dt = 2e-4
    t = 0.
    iteration = 0
    while True:
        t += dt
        iteration += 1
        robot.track_tracjectory_step(t, dt)
        if iteration % 20 == 0:
            robot.dysplay()

if __name__ == '__main__':
    main()
