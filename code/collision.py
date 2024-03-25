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

# TODO:(elisfromkirov) clean up the code

class Collision:
    def __init__(self, index_1, index_2, position, normal, distance):
        self._index_1 = index_1
        self._index_2 = index_2
        self._position = position
        self._normal = normal
        self._distance = distance

    def index_1(self):
        return self._index_1

    def index_2(self):
        return self._index_2

    def position(self):
        return self._position

    def normal(self):
        return self._normal

    def distance(self):
        return self._distance

class Robot:
    def __init__(self, model_file_path):
        # robot model parameters
        self.model, self.geometry, self.mesh = pin.buildModelsFromUrdf(model_file_path, WORKSPACE_DIR)

        # data for computation
        self.model_data = self.model.createData()
        self.geometry_data = self.geometry.createData()

        # robot vistualizer
        self.visualizer = vis.GepettoVisualizer(self.model, self.geometry, self.mesh)
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
        self.trajectory = Trajectory(omega, amplitude)

    def dysplay(self):
        self.visualizer.display(self.q)

    def crba(self):
        return pin.crba(self.model, self.data, self.q)

    def rnea(self, aq = None):
        if aq is None:
            aq = self.zeros
        return pin.rnea(self.model, self.data, self.q, self.vq, aq)

    # @param collision is pair of indices of colliding geometry objects
    # @param contacts is array of contact points for collinding geometry objects
    def getCollisionJacobian(self, collision):
        # obtain parent joint for first colliding object
        joint_1 = self.geometry.geometryObjects[collision.index_1()].parentJoint

        # transformation from first joint frame to origin frame
        oMj1 = self.model_data.oMi[joint_1]

        # obtain parent joint for second colliding object
        joint2 = self.geometry.geometryObjects[collision.snd()].parentJoint

        # transformation from second joint frame to origin frame
        oMj2 = self.model_data.oMi[joint2]

        # transformation from contact frame to origin frame
        oMc = pin.SE3(pin.Quaternion.FromTwoVectors(np.array([0,0,1]), collision.normal()).matrix(), collision.position())

        # transformation from origin frame to contact frame
        cMo = oMc.inverse()

        # transformation from first joint frame to contact frame
        cMj1 = cMo @ oMj1

        # transformation from second joint frame to contact frame
        cMj2 = cMo @ oMj2

        # frist jacobian (velocity of first geometry object in contact frame is J1 @ q)
        J1 = cMj1.action @ pin.getJointJacobian(self.model, self.model_data, joint1, pin.ReferenceFrame.LOCAL)

        # second jacobian (velocity of second geometry object in contact frame is J1 @ q)
        J2 = cMj2.action @ pin.getJointJacobian(self.model, self.model_data, joint1, pin.ReferenceFrame.LOCAL)

        # we are interested only in velocity projection to contact normal
        return (J1 - J2)[2, :]

    # collision
    # - index of first geometry object
    # - index of second geometry object
    # - position of contact
    # - normal of contact
    # - distance between geometry objects

    def compute_collisions(self, min_distance = -1e-4):
        pin.computeCollisions(self.model, self.model_data, self.geometry, self.geometry_data, self.q, False)
        pin.computeDistances(self.model, self.model_data, self.geometry, self.geometry_data, self.q)

        collisions = []

        for i, result in enumerate(zip(self.geometry_data.collisionResults, self.geometry_data.distanceResults)):
            if result[0].isCollision() and result[1].min_distance < min_distance:
                collision = self.geometry.collisionPair[i]
                contact = result[0].getContact(0)
                distance = result[1].min_distance
                collisions.append((collision.first, collision.second, contact.pos, contact.normal, distance))

        return collisions

    def track_tracjectory_step(self, t, dt):
        H = self.crba()
        c = self.rnea()

        q_des = self.trajectory.position(t)
        vq_des = self.trajectory.velocity(t)
        aq_des = self.trajectory.acceleration(t)

        tau = - self.k_p * (self.q - q_des) - self.k_d * (self.vq - vq_des) + self.k * aq_des

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
