import numpy as np
import os.path as fs
import pinocchio as pin
import pinocchio.visualize as vis

class Talos:
    def __init__(self):
        workspace_dir = fs.dirname(fs.abspath('{}/../../'.format(__file__)))
        model_file_path = '{}/example-robot-data/robots/talos_data/robots/talos_reduced.urdf'.format(workspace_dir)

        # robot model parameters
        self.model, self.collision_model, self.visual_model = pin.buildModelsFromUrdf(model_file_path, workspace_dir)

        # data for computation
        self.data = self.model.createData()

        # robot vistualizer
        self.visualizer = vis.GepettoVisualizer(self.model, self.collision_model, self.visual_model)
        self.visualizer.initViewer()
        self.visualizer.loadViewerModel("pinocchio")

        # current robot configuration
        self.q = pin.neutral(self.model)
        self.v = np.zeros(self.model.nv)
        self.a = np.zeros(self.model.nv)

    def display(self):
        self.visualizer.display(self.q)
