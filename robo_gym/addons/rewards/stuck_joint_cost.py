from gym import spaces
import pybullet as p

from ..addon import Addon


class StuckJointCost(Addon):
    def __init__(self, parent, config):
        super(StuckJointCost, self).__init__(parent, config)

        self.uid = parent.uid
        self.joint_ids = [i for i in range(p.getNumJoints(self.uid)) if p.getJointInfo(self.uid, i)[3] > -1]
        self.multiplier = config.get('multiplier', 0.1)

        joint_info = [p.getJointInfo(self.uid, i) for i in self.joint_ids]
        lower_limit = np.array([info[8] for info in joint_info])
        upper_limit = np.array([info[9] for info in joint_info])

    def reward(self):
        return -self.multiplier if np.any(np.min(np.abs(lower_limit-position), np.abs(upper_limit-position)) > 0.01) else 0.0
