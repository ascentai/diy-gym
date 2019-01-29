import pybullet as p

from ..addon import Addon
from abc import abstractmethod


class RandomizerInterface(Addon):
    def __init__(self, parent, config):
        super(RandomizerInterface, self).__init__(parent, config)

        self.uid = parent.uid

        self.joint_ids = [p.getJointInfo(self.uid, i)[0] for i in range(p.getNumJoints(self.uid))
                          if p.getJointInfo(self.uid, i)[3] > -1]
        self.joint_ids = [-1] if not self.joint_ids else self.joint_ids

    @abstractmethod
    def reset(self):
        pass
