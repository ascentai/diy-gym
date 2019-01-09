from gym import spaces
import numpy as np
from plugins.plugin import Plugin


class ReachTarget(Plugin):
    def __init__(self, parent, config):
        super(ReachTarget, self).__init__()

        self.uid = parent.uid
        self.frame_id = [p.getJointInfo(self.uid, i)[1].decode('utf-8') for i in range(p.getNumJoints(self.uid))].index(config.get('frame'))

        self.target_position = np.array(config.get('target_xyz')))
        self.target_orientation = config.get('target_rpy')
        self.multiplier = config.get('multiplier', 1.0)
        self.tolerance = config.get('tolerance', 0.01)

        self.observation_space = spaces.Dict({
            'target_position': spaces.Box(-10, 10, shape=(3,), dtype='float32'),
            'achieved_position': spaces.Box(-10, 10, shape=(3,), dtype='float32'),
            'target_orientation': spaces.Box(-10, 10, shape=(3,), dtype='float32'),
            'achieved_orientation': spaces.Box(-10, 10, shape=(3,), dtype='float32')
        })

    def observation(self):
        link_state = p.getLinkState(self.uid, self.frame_id)

        obs = {}
        obs['target_position'] = self.target_orientation
        obs['achieved_position'] = link_state[0]
        obs['target_orientation'] = self.target_orientation
        obs['achieved_orientation'] = p.getEulerFromQuaternion(link_state[1])

    def dist(self, obs):
        obs = self.observation()
        q_target, q_achieved = np.array(p.getQuaternionFromEuler(obs['target_orientation'])), np.array(p.getQuaternionFromEuler(obs['achieved_orientation']))
        return np.linalg.norm(np.array(obs['target_position']) - np.array(obs['achieved_position'])) + np.acos(2 * q_target.dot(q_achieved)**2 - 1)

    def reward(self):
        return self.dist() * self.multiplier

    def is_terminal(self):
        return self.dist() < self.tolerance
