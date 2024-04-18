# proper force calculation #
# I did that #

import pinocchio as pin
import numpy as np

from base import get_model_file_path

def main():
    model_file_path, model_dir = get_model_file_path('talos_data/robots/talos_reduced.urdf')

    model, collision_model, visual_model = pin.buildModelsFromUrdf(
        model_file_path,
        model_dir,
        pin.JointModelFreeFlyer()
    )

    data = model.createData()

    q = pin.neutral(model)
    v = np.zeros(model.nv)
    a = np.ones(model.nv)
    z = np.zeros(model.nv)

    pin.computeJointJacobians(model, data, q)

    H = pin.crba(model, data, q)

    c = pin.rnea(model, data, q, v, z)

    J = np.vstack([pin.getJointJacobian(model, data, joint, pin.LOCAL) for joint in range(0, model.njoints)])

    # f = np.zeros(J.shape[0])
    f = np.ones(J.shape[0])

    u = H @ a + c - J.T @ f
    # print(u)

    f_ext = [pin.Force.Zero() for joint in range(0, model.njoints)]
    for joint in range(0, model.njoints):
        f_ext[joint].linear = f[joint * 6 : joint * 6 + 3]
        f_ext[joint].angular = f[joint * 6 + 3 : joint * 6 + 6]
    # print(f_ext)

    a = pin.aba(model, data, q, v, u, f_ext)
    print(a)

if __name__ == '__main__':
    main()
