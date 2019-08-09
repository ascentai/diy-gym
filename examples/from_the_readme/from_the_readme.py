#!/usr/bin/env python

import numpy as np
import os
from diy_gym import DIYGym

if __name__ == '__main__':

    config_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'from_the_readme.yaml')

    env = DIYGym(config_file)

    observation = env.reset()

    while True:
        action = env.action_space.sample()

        action['robot']['controller']['linear'][:] = observation['from_the_readme']['where_is_r2d2']['position'] * 0.3

        observation, reward, terminal, info = env.step(action)

        if terminal['from_the_readme']['grab_r2d2']:
            observation = env.reset()
