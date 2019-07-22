import unittest
import os
import numpy as np
from diy_gym import DIYGym
from diy_gym.utils import flatten, unflatten


class TestEnvironment(unittest.TestCase):
    def setUp(self):
        config_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'basic_env.yaml')
        self.env = DIYGym(config_file)

    def test_flatten_unflatten(self):
        action = self.env.action_space.sample()

        flattened_action = flatten(action)
        unflattened_action = unflatten(flattened_action, self.env.action_space)

        self.assertTrue(np.all(action['red_marble']['force'] == unflattened_action['red_marble']['force']))
        self.assertTrue(np.all(action['blue_marble']['force'] == unflattened_action['blue_marble']['force']))


if __name__ == '__main__':
    unittest.main()
