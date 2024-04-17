# import numpy as np
# import pinocchio as pin
# import pinocchio.visualize as vis

# class Robot():
#     def __init__(self, model_file_path, workspace_dir):
#         self.model, self.collision_model, self.visual_model = pin.buildModelsFromUrdf(
#             model_file_path,
#             workspace_dir,
#             pin.JointModelFreeFlyer()
#         )

#         # data for computation
#         self.data = self.model.createData()

#         # robot vistualizer
#         self.visualizer = vis.GepettoVisualizer(self.model, self.collision_model, self.visual_model)
#         self.visualizer.initViewer()
#         self.visualizer.loadViewerModel("pinocchio")

#         # floor
#         self.visualizer.viewer.gui.addFloor('hpp-gui/floor')
#         self.visualizer.viewer.gui.setScale('hpp-gui/floor', [0.5, 0.5, 0.5])
#         self.visualizer.viewer.gui.setStaticTransform('hpp-gui/floor', [0.0, 0.0, -1.085, 1.0, 0.0, 0.0, 0.0])
#         self.visualizer.viewer.gui.setColor('hpp-gui/floor', [0.7, 0.7, 0.7, 1.])
#         self.visualizer.viewer.gui.setLightingMode('hpp-gui/floor', 'OFF')

#         # current robot configuration
#         self.q = pin.neutral(self.model)
#         self.v = np.zeros(self.model.nv)
#         self.a = np.zeros(self.model.nv)

#     def aba(self, u, f = None):
#         if f == None:
#             return pin.aba(self.model, self.data, self.q, self.v, u, f)
#         return pin.aba(self.model, self.data, self.q, self.v, u)

#     def rnea(self, a = None):
#         if a == None:
#             return pin.rnea(self.model, self.data, self.q, self.v, self.zeros)
#         pin.rnea(self.model, self.data, self.q, self.v, a)

