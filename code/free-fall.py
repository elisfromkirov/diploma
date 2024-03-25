import numpy as np
import os.path as fs
import pinocchio as pin
import pinocchio.visualize as vis

WORKSPACE_DIR = fs.dirname(fs.abspath('{}/../'.format(__file__)))

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

    def dysplay(self):
        self.visualizer.display(self.q)

    def crba(self):
        return pin.crba(self.model, self.data, self.q)

    def rnea(self, aq = None):
        if aq is None:
            aq = self.zeros
        return pin.rnea(self.model, self.data, self.q, self.vq, aq)

    def free_fall_step(self, dt):
        H = self.crba()
        c = self.rnea()
        tau = self.zeros

        self.aq = np.linalg.pinv(H) @ (tau - c)
        self.vq += self.aq * dt
        self.q = pin.integrate(self.model, self.q, self.vq * dt)

def main():
    robot = Robot(f'{WORKSPACE_DIR}/example-robot-data/robots/romeo_description/urdf/romeo.urdf')
    # robot = Robot(f'{WORKSPACE_DIR}/example-robot-data/robots/ur_description/urdf/ur5_robot.urdf')
    robot.dysplay()

    iteration = 0
    while True:
        iteration += 1
        robot.free_fall_step(2e-5)
        if iteration % 20 == 0:
            robot.dysplay()

if __name__ == '__main__':
    main()
