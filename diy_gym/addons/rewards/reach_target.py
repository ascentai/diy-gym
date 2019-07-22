import numpy as np
import pybullet as p
from diy_gym.addons.addon import Addon


class ReachTarget(Addon):
    def __init__(self, parent, config):
        super(ReachTarget, self).__init__(parent, config)

        self.source_model = parent.models[config.get('source_model')]
        self.target_model = parent.models[config.get('target_model')]

        self.source_frame_id = self.source_model.get_frame_id(config.get('source_frame')) if 'source_frame' in config else -1
        self.target_frame_id = self.target_model.get_frame_id(config.get('target_frame')) if 'target_frame' in config else -1

        self.multiplier = config.get('multiplier', 1.0)
        self.tolerance = config.get('tolerance', 0.05)

    def dist(self):
        source_xyz = p.getLinkState(self.source_model.uid, self.source_frame_id)[4] if self.source_frame_id >= 0 else p.getBasePositionAndOrientation(self.source_model.uid)[0]
        target_xyz = p.getLinkState(self.target_model.uid, self.target_frame_id)[4] if self.target_frame_id >= 0 else p.getBasePositionAndOrientation(self.target_model.uid)[0]
        return np.linalg.norm(np.array(target_xyz) - source_xyz)

    def reward(self):
        return -self.dist() * self.multiplier

    def is_terminal(self):
        return self.dist() < self.tolerance
