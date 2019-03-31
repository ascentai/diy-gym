import random
import pybullet as p
from ..addon import Addon


class ExternalForce(Addon):
    def __init__(self, parent, config):
        super(ExternalForce, self).__init__(parent, config)

        self.uid = parent.uid
        joint_name = config.get('joint', -1)
        if joint_name == -1:
            self.link_idx = joint_name
        else:
            self.link_idx = [p.getJointInfo(self.uid, i)[1].decode('utf-8') for i in
                             range(p.getNumJoints(self.uid))].index(joint_name)
        self.force_applied = config.get('force_applied', [0, 0, 0])
        self.position_applied = config.get('position_applied', [0, 0, 0])
        self.coordinate_frame = config.get('coordinate_frame', p.WORLD_FRAME)
        self.random_forces = config.get('random', False)

        self.reset()

    def reset(self):
        if self.random_forces:
            self.force_applied = [random.randrange(-500, 500, 50) for i in range(3)]
            self.position_applied = [random.random() for i in range(3)]

        p.applyExternalForce(self.uid, self.link_idx, self.force_applied, self.position_applied, self.coordinate_frame)
