import numpy as np
import pybullet as p
from gym import spaces
from diy_gym.addons.addon import Addon


class ForceTorqueSensor(Addon):

    def __init__(self, parent, config, link_name=None, force_enabled=False):
        super(ObjectStateSensor, self).__init__(parent, config)

        self.uid = parent.uid
        self.frame_id = parent.get_frame_id(config.get('frame')) if 'frame' in config else -1

        p.enableJointForceTorqueSensor(self.uid, self.frame_id, enableSensor=True)

        self.observation_space = spaces.Dict({
            'force'   : spaces.Box(-10, 10, shape=(3,), dtype='float32'),
            'torque': spaces.Box(-10, 10, shape=(3,), dtype='float32'),
        })

    def observe(self):
        state = p.getJointState(self.uid, self.frame_id)
        return {'force' : state[2][:3], 'torque': state[2][3:]}
