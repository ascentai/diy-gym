import pybullet as p
import numpy as np
from gym import spaces
from diy_gym.addons.addon import Addon


class JointStateSensor(Addon):
    def __init__(self, parent, config):
        super(JointStateSensor, self).__init__(parent, config)

        self.uid = parent.uid

        if 'joints' in config:
            self.joint_ids = [[p.getJointInfo(self.uid, i)[1].decode('utf-8')
                               for i in range(p.getNumJoints(self.uid))].index(joint) for joint in config.get('joints')]
        else:
            self.joint_ids = [i for i in range(p.getNumJoints(self.uid)) if p.getJointInfo(self.uid, i)[3] > -1]

        self.include_velocity = config.get('include_velocity', True)
        self.include_effort = config.get('include_effort', False)

        joint_info = [p.getJointInfo(self.uid, i) for i in self.joint_ids]
        joint_position_lower_limit = np.array([info[8] for info in joint_info])
        joint_position_upper_limit = np.array([info[9] for info in joint_info])

        self.observation_space = spaces.Dict(
            {'position': spaces.Box(low=joint_position_lower_limit, high=joint_position_upper_limit, dtype='float32')})

        if self.include_velocity:
            joint_velocity_limit = np.array([info[11] for info in joint_info])
            self.observation_space.spaces['velocity'] = spaces.Box(low=-joint_velocity_limit,
                                                                   high=joint_velocity_limit,
                                                                   dtype='float32')

        if self.include_effort:
            torque_limit = np.array([info[10] for info in joint_info])
            self.observation_space.spaces['effort'] = spaces.Box(low=-torque_limit, high=torque_limit, dtype='float32')

    def observe(self):
        joint_states = p.getJointStates(self.uid, self.joint_ids)

        obs = {'position': [state[0] for state in joint_states]}

        if self.include_velocity:
            obs['velocity'] = [state[1] for state in joint_states]

        if self.include_effort:
            obs['effort'] = [state[3] for state in joint_states]

        return obs
