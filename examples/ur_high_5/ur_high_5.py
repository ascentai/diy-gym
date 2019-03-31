#!/usr/bin/env python

import numpy as np
import pybullet as p

from robo_gym import RoboGym

if __name__ == '__main__':

    env = RoboGym('ur_high_5.yaml')

    target_pos = [0 for i in range(3)]
    target_theta = [0 for i in range(3)]

    duck_obs = p.getBasePositionAndOrientation(env.models['duck'].uid)

    while True:
        action = env.action_space.sample()

        action['ur5_l']['position_controller']['position'] = np.array([duck_obs[0][0] - 0.25, duck_obs[0][1], duck_obs[0][2] + 0.3])
        action['ur5_l']['position_controller']['orientation'] = np.array([duck_obs[1][0], duck_obs[1][1] - 0.005, duck_obs[1][2]])

        action['ur5_r']['controller']['position'] = np.array([duck_obs[0][0], duck_obs[0][1], duck_obs[0][2] + 0.3])
        action['ur5_r']['controller']['orientation'] = np.array([duck_obs[1][0], duck_obs[1][1] + 0.005, duck_obs[1][2]])

        observation, reward, terminal, info = env.step(action)

        if terminal:
            print('slap!')
            env.reset()
