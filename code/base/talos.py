import example_robot_data as robex
import numpy as np
import os
import pinocchio as pin
import pinocchio.visualize as vis

def get_model_file_path(model_file_path):
  model_dir = robex.getModelPath(model_file_path)
  return os.path.join(model_dir, model_file_path.strip('/')), os.path.dirname(os.path.dirname(model_dir))

class Talos:
    def __init__(self, floating_base = True):
        model_file_path, model_dir = get_model_file_path('talos_data/robots/talos_reduced.urdf')

        self.model, self.collision_model, self.visual_model = pin.buildModelsFromUrdf(
            model_file_path,
            model_dir,
            pin.JointModelFreeFlyer() if floating_base else None
        )

        self.data = self.model.createData()

        try:
            self.visualizer = vis.GepettoVisualizer(self.model, self.collision_model, self.visual_model)
            self.visualizer.initViewer()
            self.visualizer.loadViewerModel("pinocchio")

            self.visualizer.viewer.gui.addFloor('hpp-gui/floor')
            self.visualizer.viewer.gui.setScale('hpp-gui/floor', [0.5, 0.5, 0.5])
            self.visualizer.viewer.gui.setStaticTransform('hpp-gui/floor', [0.0, 0.0, -1.085, 1.0, 0.0, 0.0, 0.0])
            self.visualizer.viewer.gui.setColor('hpp-gui/floor', [0.7, 0.7, 0.7, 1.])
            self.visualizer.viewer.gui.setLightingMode('hpp-gui/floor', 'OFF')
        except:
          quit('Seems like you forgot to launch gepetto-gui! Please open another terminal and run gepetto-gui in it.')

        self.q = pin.neutral(self.model)
        self.v = np.zeros(self.model.nv)
        self.a = np.zeros(self.model.nv)

        if floating_base:
            self.projection = np.zeros((self.model.nv, self.model.nq))
            for i in range(3):
                self.projection[i][i + 1] = 1
            for i in range(7, self.model.nv):
                self.projection[i][i + 1] = 1

        self.zero_a = np.zeros(self.model.nv)

    def apply_forward_dynamics(self, u, f, dt):
        assert u.ndim == 1 and u.shape[0] == self.model.nv
        assert f.ndim == 1 and f.shape[0] == self.model.njoints * 6

        f_ext = [pin.Force.Zero() for joint in range(0, self.model.njoints)]
        for joint in range(0, self.model.njoints):
            f_ext[joint].linear = f[joint * 6 : joint * 6 + 3]
            f_ext[joint].angular = f[joint * 6 + 3 : joint * 6 + 6]

        self.a = pin.aba(self.model, self.data, self.q, self.v, u, f_ext)
        self.v += self.a * dt
        self.q = pin.integrate(self.model, self.q, self.v * dt)

    def compute_joint_space_inertia(self):
        return pin.crba(self.model, self.data, self.q)

    def compute_non_linear_term(self):
        return pin.rnea(self.model, self.data, self.q, self.v, self.zero_a)

    def compute_joint_jacobians(self):
        pin.computeJointJacobians(self.model, self.data, self.q)

    def get_jacobian(self):
        return np.vstack([pin.getJointJacobian(self.model, self.data, joint, pin.LOCAL) for joint in range(0, self.model.njoints)])

    def get_frame_index(self, frame_name):
        return self.model.getFrameId(frame_name)

    def get_frame_jacobian(self, frame_index):
        return pin.getFrameJacobian(self.model, self.data, frame_index, pin.LOCAL)

    def get_frame_jacobian_variation(self, frame_index):
        return pin.getFrameJacobianTimeVariation(self.model, self.data, frame_index, pin.LOCAL)

    def update_frame_placements(self):
        pin.updateFramePlacements(self.model, self.data)

    def display(self):
        self.visualizer.display(self.q)
