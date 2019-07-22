import numpy as np
import pybullet as p
from diy_gym.addons.addon import Addon


class Respawn(Addon):
    def __init__(self, parent, config):
        super(Respawn, self).__init__(parent, config)

        self.uid = parent.uid
        self.origin = p.getBasePositionAndOrientation(self.uid)[0]
        self.position_range = config.get('position_range', [0., 0., 0.])
        self.rotation_range = config.get('rotation_range', [0., 0., 0.])
        self.joint_range = config.get('joint_range', [0., 0., 0.])

        self.once = config.get('once', False)

        self.pose = self.generate_pose()

        self.reset()

    def reset(self):
        if not self.once:
            self.pose = self.generate_pose()

        p.resetBasePositionAndOrientation(self.uid, *self.pose)

    def generate_pose(self):
        return ((np.random.random(3) - 0.5) * self.position_range + self.origin,
                p.getQuaternionFromEuler((np.random.random(3) - 0.5) * self.rotation_range))
