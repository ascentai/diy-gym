import unittest
import os
import numpy as np
from diy_gym import DIYGym


class TestEnvironment(unittest.TestCase):
    def setUp(self):
        config_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'basic_env.yaml')
        self.env = DIYGym(config_file)

    def test_load_environment(self):
        self.assertTrue('plane' in self.env.models)
        self.assertTrue('red_marble' in self.env.models)
        self.assertTrue('green_marble' in self.env.models)
        self.assertTrue('blue_marble' in self.env.models)

    def test_spaces(self):
        self.assertTrue('force' in self.env.action_space['blue_marble'].spaces)
        self.assertTrue('camera' in self.env.observation_space['basic_env'].spaces)
        self.assertTrue('pose' in self.env.observation_space['green_marble'].spaces)

    def test_episode(self):
        observation = self.env.reset()
        initial_position = observation['green_marble']['pose']['position']

        # try to run the blue marble into the other two
        for _ in range(500):
            observation, _, _, _ = self.env.step({'blue_marble': {'force': [0, -100, 0]}})

        final_position = observation['green_marble']['pose']['position']

        # check that the green marble has moved
        self.assertNotAlmostEqual(np.linalg.norm(initial_position), np.linalg.norm(final_position), places=0)

        observation = self.env.reset()
        reset_position = observation['green_marble']['pose']['position']

        # check that the green marble has been reset back to its starting position
        self.assertAlmostEqual(np.linalg.norm(initial_position), np.linalg.norm(reset_position), places=1)


if __name__ == '__main__':
    unittest.main()
