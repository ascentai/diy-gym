import numpy as np
from robo_gym import RoboGym

if __name__ == '__main__':

    env = RoboGym('robot_room.xml')

    action = env.action_space.sample()

    action['robot']['controller']['position'] = np.array([0.01, 0.0, -0.01])
    action['robot']['controller']['orientation'] = np.array([-0.05, 0.0, 0.0])

    cumulative_reward = 0

    while True:
        observation, reward, terminal, info = env.step(action)

        cumulative_reward += reward['robot']['be_lazy']

        if terminal['environment']['timer']:
            print('Time is up! cumulative electricity cost was: %f' % cumulative_reward)
            env.reset()
            cumulative_reward = 0