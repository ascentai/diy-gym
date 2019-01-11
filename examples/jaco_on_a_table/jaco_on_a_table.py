import numpy as np
from robo_gym import RoboGym

if __name__ == '__main__':

    env = RoboGym('jaco_on_a_table.xml')

    action = env.action_space.sample()

    action['robot']['controller']['position'] = np.array([0.005, 0.005, -0.005])
    action['robot']['controller']['orientation'] = np.array([-0.02, 0.02, -0.02])

    while True:
        observation, reward, terminal, info = env.step(action)