#!/usr/bin/env python
import argparse
import os
import time
import torch
import numpy as np
import matplotlib.pyplot as plt

import pybullet as p

from robo_gym import RoboGym

if __name__ == '__main__':

    env = RoboGym('pick_and_place.yaml')

    while True:
        action = env.action_space.sample()

        action['ur5']['force_controller']['wrench'] = np.array([150.0, 210.0, 0.0, 0.0, 0.0, 0.0])
        action['ur5']['force_controller']['wrench'] = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        action['ur5']['2f_gripper_controller']['position'] = 0.0

        observation, reward, terminal, info = env.step(action)

        print(observation['ur5']['load_sensor']['force_measured'])

        if terminal:
            env.reset()
            print('\nEnd of episode...\n')
