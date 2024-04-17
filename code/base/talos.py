import numpy as np
import os.path as fs
import pinocchio as pin
import pinocchio.visualize as vis

class Talos:
    def __init__(self):
        workspace_dir = fs.dirname(fs.abspath('{}/../../'.format(__file__)))
        model_file_path = '{}/example-robot-data/robots/talos_data/robots/talos_reduced.urdf'.format(workspace_dir)

        self.model, self.collision_model, self.visual_model = pin.buildModelsFromUrdf(
            model_file_path,
            workspace_dir,
            pin.JointModelFreeFlyer()
        )

        self.data = self.model.createData()

        self.visualizer = vis.GepettoVisualizer(self.model, self.collision_model, self.visual_model)
        self.visualizer.initViewer()
        self.visualizer.loadViewerModel("pinocchio")

        self.visualizer.viewer.gui.addFloor('hpp-gui/floor')
        self.visualizer.viewer.gui.setScale('hpp-gui/floor', [0.5, 0.5, 0.5])
        self.visualizer.viewer.gui.setStaticTransform('hpp-gui/floor', [0.0, 0.0, -1.085, 1.0, 0.0, 0.0, 0.0])
        self.visualizer.viewer.gui.setColor('hpp-gui/floor', [0.7, 0.7, 0.7, 1.])
        self.visualizer.viewer.gui.setLightingMode('hpp-gui/floor', 'OFF')

        self.q = pin.neutral(self.model)
        self.v = np.zeros(self.model.nv)
        self.a = np.zeros(self.model.nv)

        self.projection = np.zeros((self.model.nv, self.model.nq))
        for i in range(3):
            self.projection[i][i + 1] = 1
        for i in range(7, self.model.nv):
            self.projection[i][i + 1] = 1

        self.zeros = np.zeros(self.model.nv)

    def display(self):
        self.visualizer.display(self.q)

    def describe(self):
        nq = self.model.nq
        print(f'nq = {nq}')

        nv = self.model.nv
        print(f'nv = {nv}')

        print('joints = [')
        for i, name in enumerate(self.model.names):
            print(f'  [{i:02}] = {name}')
        print(']')