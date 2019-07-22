import math
import random

import pybullet as p
from diy_gym.addons.addon import Addon


class DynamicsRandomizer(Addon):
    def __init__(self, parent, config):
        super(DynamicsRandomizer, self).__init__(parent, config)

        self.uid = parent.uid

        self.joint_ids = [
            p.getJointInfo(self.uid, i)[0] for i in range(p.getNumJoints(self.uid))
            if p.getJointInfo(self.uid, i)[3] > -1
        ]
        self.joint_ids = [-1] if not self.joint_ids else self.joint_ids

        self.mass_range = config.get('mass_range', [0.25, 4.0])
        self.damping_range = config.get('damping_range', [0.2, 20])
        self.reset()

    def reset(self):
        # reference [https://arxiv.org/pdf/1710.06537.pdf] Table 1 for parameter ranges
        for joint_id in self.joint_ids:
            new_mass_value = math.log(random.uniform(self.mass_range[0], self.mass_range[1])) * p.getDynamicsInfo(
                self.uid, joint_id)[0]
            new_joint_damping_value = math.log(random.uniform(
                self.damping_range[0], self.damping_range[1])) * p.getJointInfo(self.uid, joint_id)[6]

            p.changeDynamics(self.uid, joint_id, mass=new_mass_value, angularDamping=new_joint_damping_value)
