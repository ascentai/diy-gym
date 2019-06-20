import numpy as np

from diy_gym.addons.addon import Addon


class TimePenalty(Addon):
    def __init__(self, parent, config):
        super(TimePenalty, self).__init__(parent, config)
        self.penalty = config.get('penalty', -1)

    def reward(self):
        return self.penalty

