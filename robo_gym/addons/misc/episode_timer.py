import numpy as np
from gym import spaces
from ..addon import Addon


class EpisodeTimer(Addon):
    def __init__(self, parent, config):
        super(EpisodeTimer, self).__init__()

        self.action_space = spaces.Box(low=-1, high=1, shape=(0,), dtype=np.uint8)
        self.step_timer = 0
        self.max_steps = config.get('max_steps')

    def update(self, action):
        self.step_timer += 1

    def reset(self):
        self.step_timer = 0

    def is_terminal(self):
        return self.step_timer >= self.max_steps
