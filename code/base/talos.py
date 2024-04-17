import pinocchio as pin
import pinocchio.visualize as vis
import numpy as np

from .utility import get_model_file_path

class Talos:
    def __init__(self):
        # model file path
        model_file_path, model_dir = get_model_file_path('talos_data/robots/talos_reduced.urdf')

        # robot model parameters
        self.model, self.collision_model, self.visual_model = pin.buildModelsFromUrdf(model_file_path, model_dir)

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
