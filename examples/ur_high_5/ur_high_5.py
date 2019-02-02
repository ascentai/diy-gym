#!/usr/bin/env python

import numpy as np
from robo_gym import RoboGym

if __name__ == '__main__':

    env = RoboGym('ur_high_5.yaml')

    while True:
        action = env.action_space.sample()

        action['ur5_l']['position_controller']['position'] = np.array([0.005, 0.0, 0.0])
        action['ur5_l']['position_controller']['orientation'][:] = 0

        action['ur5_r']['controller']['position'] = np.array([-0.005, 0.0, 0.0])
        action['ur5_r']['controller']['orientation'][:] = 0

        observation, reward, terminal, info = env.step(action)

        if terminal:
            print('slap!')
            env.reset()
