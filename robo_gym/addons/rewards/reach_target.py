from gym import spaces
import numpy as np
import pybullet as p

from ..addon import Addon


class ReachTarget(Addon):
    def __init__(self, parent, config):
        super(ReachTarget, self).__init__()

        self.uid = parent.uid
        self.frame_id = [p.getJointInfo(self.uid, i)[1].decode('utf-8') for i in range(p.getNumJoints(self.uid))].index(config.get('frame'))

        self.target_position = config.get('target_position')
        self.multiplier = config.get('multiplier', 1.0)
        self.tolerance = config.get('tolerance', 0.1)
        self.use_ori = config.has_key('target_rpy')

        self.observation_space = spaces.Dict({
            'target_position': spaces.Box(-10, 10, shape=(3,), dtype='float32'),
            'achieved_position': spaces.Box(-10, 10, shape=(3,), dtype='float32')
        })

        if self.use_ori:
            self.target_orientation = config.get('target_orientation')
            self.observation_space.spaces.update({
                'target_orientation': spaces.Box(-10, 10, shape=(3,), dtype='float32'),
                'achieved_orientation': spaces.Box(-10, 10, shape=(3,), dtype='float32')
            })

    def observation(self):
        link_state = p.getLinkState(self.uid, self.frame_id)

        obs = {}
        obs['target_position'] = self.target_position
        obs['achieved_position'] = link_state[0]

        if self.use_ori:
            obs['target_orientation'] = self.target_orientation
            obs['achieved_orientation'] = p.getEulerFromQuaternion(link_state[1])

        return obs

    def dist(self):
        obs = self.observation()
        d = np.linalg.norm(np.array(obs['target_position']) - np.array(obs['achieved_position']))

        if self.use_ori:
            q_target = np.array(p.getQuaternionFromEuler(obs['target_orientation']))
            q_achieved = np.array(p.getQuaternionFromEuler(obs['achieved_orientation']))
            d += np.acos(2 * q_target.dot(q_achieved)**2 - 1)

        return d

    def reward(self):
        return self.dist() * self.multiplier

    def is_terminal(self):
        return self.dist() < self.tolerance
