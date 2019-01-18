import numpy as np
import pybullet as p
from ..addon import Addon


class RandomRespawn(Addon):
    def __init__(self, parent, config):
        super(RandomRespawn, self).__init__()

        self.uid = parent.uid
        self.origin = p.getBasePositionAndOrientation(self.uid)[0]
        self.position_range = config.get('position_range', [0., 0., 0.])
        self.rotation_range = config.get('rotation_range', [0., 0., 0.])
        self.reset()

    def reset(self):
        p.resetBasePositionAndOrientation(
            self.uid,
            (np.random.random(3) - 0.5) * self.position_range + self.origin,
            p.getQuaternionFromEuler((np.random.random(3) - 0.5) * self.rotation_range)
        )
