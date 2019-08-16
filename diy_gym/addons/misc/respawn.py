import numpy as np
import pybullet as p
from diy_gym.addons.addon import Addon


class Respawn(Addon):
    def __init__(self, parent, config):
        super(Respawn, self).__init__(parent, config)

        self.uid = parent.uid
        self.initial_pose = p.getBasePositionAndOrientation(self.uid)
        self.position_range = config.get('position_range', [0., 0., 0.])
        self.rotation_range = config.get('rotation_range', [0., 0., 0.])

        self.once = config.get('once', False)

        self.pose = self.generate_pose()

        self.reset()

    def quaternion_multiply(self, quaternion1, quaternion0):
        ''' Return multiplication of two quaternions. '''
        x0, y0, z0, w0 = quaternion0
        x1, y1, z1, w1 = quaternion1
        return np.array((
            x1*w0 + y1*z0 - z1*y0 + w1*x0,
            -x1*z0 + y1*w0 + z1*x0 + w1*y0,
            x1*y0 - y1*x0 + z1*w0 + w1*z0,
            -x1*x0 - y1*y0 - z1*z0 + w1*w0), dtype=np.float64)

    def reset(self):
        if not self.once:
            self.pose = self.generate_pose()

        p.resetBasePositionAndOrientation(self.uid, *self.pose)

    def generate_pose(self):
        return ((np.random.random(3) - 0.5) * self.position_range + self.initial_pose[0],
                self.quaternion_multiply(self.initial_pose[1], p.getQuaternionFromEuler((np.random.random(3) - 0.5) * self.rotation_range)))
