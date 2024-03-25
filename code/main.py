# TODO: tasks
#  1. make a trajectory to follow
#  2. implement Momentum Control for Balance
#  3. implement Diploma Control

import numpy as np
import os.path as fs
import pinocchio as pin
import pinocchio.visualize as vis

WORKSPACE_DIR = fs.dirname(fs.abspath('{}/../'.format(__file__)))

# TODO: We also need a control parameters for all our weights

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

        # joint info
        for i, name in enumerate(self.model.names):
            print(f'joint-{i} {name}')

        njoints = self.model.njoints
        print(f'njoints={njoints}')

        nq = self.model.nq
        print(f'nq={nq}')

        nv = self.model.nv
        print(f'nv={nv}')

        nqs0 = self.model.nqs[0]
        print(f'nqs0={nqs0}')

        nqs1 = self.model.nqs[1]
        print(f'nqs1={nqs1}')

        self.q[2] = 0.875

    def dysplay(self):
        self.visualizer.display(self.q)

    def update_squat_animation(self, t, period = 20.0):
        phi = 2 * np.pi * t / period

        self.q[2] = 0.875 - 0.625 * (1.0 - np.cos(np.abs(np.sin(phi))))

        self.q[7 + 2] = -1.0 * np.abs(np.sin(phi))
        self.q[7 + 3] = 2.0 * np.abs(np.sin(phi))
        self.q[7 + 4] = -1.0 * np.abs(np.sin(phi))

        self.q[7 + 8] = -1.0 * np.abs(np.sin(phi))
        self.q[7 + 9] = 2.0 * np.abs(np.sin(phi))
        self.q[7 + 10] = -1.0 * np.abs(np.sin(phi))

def main():
    robot = Robot(f'{WORKSPACE_DIR}/example-robot-data/robots/romeo_description/urdf/romeo.urdf')
    robot.dysplay()
    dt = 2e-4
    t = 0.0
    iteration = 0
    while True:
        t += dt
        iteration += 1
        robot.update_squat_animation(t)
        if iteration % 20 == 0:
            robot.dysplay()

if __name__ == '__main__':
    main()
