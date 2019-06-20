#!/usr/bin/env python

import numpy as np
import os
import pybullet as p

from diy_gym import DIYGym

if __name__ == '__main__':

    config_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'r2d2_maze.yaml')
    env = DIYGym(config_file)

    speed = 10.

    key_and_delta = {
        p.B3G_LEFT_ARROW:  [-1,-1, 1, 1],
        p.B3G_RIGHT_ARROW: [ 1, 1,-1,-1],
        p.B3G_UP_ARROW:    [-1,-1,-1,-1],
        p.B3G_DOWN_ARROW:  [ 1, 1, 1, 1]}

    action = env.action_space.sample()

    while True:
        action['r2d2']['wheel_driver'][:] = 0

        keys = p.getKeyboardEvents()

        if 32 in keys and keys[32] & p.KEY_WAS_RELEASED:
            env.reset()

        for key, delta in key_and_delta.items():
            if key in keys and keys[key] & p.KEY_IS_DOWN:
                action['r2d2']['wheel_driver'] += np.array(delta) * speed 

        observation, reward, terminal, info = env.step(action)
