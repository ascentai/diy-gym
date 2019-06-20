#!/usr/bin/env python

import numpy as np
import os
from diy_gym import DIYGym

if __name__ == '__main__':

    config_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'ur_high_5.yaml')

    env = DIYGym(config_file)

    observation = env.reset()

    while True:
        action = env.action_space.sample()
        action['ur5_l']['controller']['linear'][:] = observation['ur_high_5']['distance_to_target']['position'] * 0.3
        action['ur5_r']['controller']['linear'][:] = -observation['ur_high_5']['distance_to_target']['position'] * 0.3
        action['ur5_l']['controller']['rotation'][:] = 0
        action['ur5_r']['controller']['rotation'][:] = 0

        observation, reward, terminal, info = env.step(action)

        if terminal:
            print('slap!')
            observation = env.reset()
