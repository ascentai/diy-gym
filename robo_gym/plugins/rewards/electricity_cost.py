from gym import spaces
from ..plugin import Plugin
import pybullet as p

class ElectricityCost(Plugin):
    def __init__(self, parent, config):
        super(ElectricityCost, self).__init__()

        self.multiplier = config.get('multiplier', 1.0)
        self.uid = parent.uid
        self.joint_ids = [i for i in range(p.getNumJoints(self.uid)) if p.getJointInfo(self.uid, i)[3] > -1]

    def reward(self):
        return -sum(state[3] for state in p.getJointStates(self.uid, self.joint_ids)) * self.multiplier