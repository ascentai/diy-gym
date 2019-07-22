from gym import spaces
import numpy as np
from diy_gym.addons.addon import Addon
import pybullet as p


class ElectricityCost(Addon):
    def __init__(self, parent, config):
        super(ElectricityCost, self).__init__(parent, config)

        self.uid = parent.uid
        self.multiplier = config.get('multiplier', 1.0)
        self.joint_ids = [i for i in range(p.getNumJoints(self.uid)) if p.getJointInfo(self.uid, i)[3] > -1]

    def reward(self):
        torque = np.array([state[3] for state in p.getJointStates(self.uid, self.joint_ids)])
        velocity = np.array([state[1] for state in p.getJointStates(self.uid, self.joint_ids)])
        return -np.abs(torque * velocity).sum() * self.multiplier
