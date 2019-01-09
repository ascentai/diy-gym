import gym
from gym import spaces
from gym.utils import seeding
import pybullet as p
import numpy as np

from config import Configuration
from model import Model
from plugins.plugin import Plugin, Receptor


class RoboGym(gym.Env, Receptor):
    def __init__(self, config_file):
        """Initialize the gym"""
        gym.Env.__init__(self)
        Receptor.__init__(self)

        config = Configuration.from_file(config_file)

        self.sum_rewards = config.get('sum_rewards', True)
        self.sum_terminals = config.get('sum_terminals', True)
        self.sub_steps = config.get('substeps', 200)

        if config.get('render', True):
            distance = config.get('camera_distance', 2.0)
            yaw = config.get('camera_yaw', 180)
            pitch = config.get('camera_pitch', -41)
            target_position = config.get('camera_target_position', [0.0, 0.20, 0.50])
            p.connect(p.GUI)
            p.resetDebugVisualizerCamera(distance, yaw, pitch, target_position)
        else:
            cId = p.connect(p.SHARED_MEMORY)
            if cId < 0: p.connect(p.DIRECT)

        p.resetSimulation()
        p.setPhysicsEngineParameter(numSolverIterations=150)

        gravity = config.get('gravity', [0.0, 0.0, -9.81])
        p.setGravity(gravity[0], gravity[1], gravity[2])

        self.models = {child.attributes['name']: Model(child) for child in config.find_all('model')}
        self.plugins = {child.attributes['name']: Plugin.factory(child.attributes['type'], self, child) for child in config.find_all('plugin')}
        self.receptors = {**self.models, 'environment': self}

        self.observation_space, self.action_space = spaces.Dict({}), spaces.Dict({})

        for name, receptor in self.receptors.items():
            obs_space, act_space = receptor.build_spaces()
            if len(obs_space.spaces): self.observation_space.spaces[name] = obs_space
            if len(act_space.spaces): self.action_space.spaces[name] = act_space

        self.seed()
        self.reset()

    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def reset(self):
        for receptor in self.receptors.values():
            receptor.reset_plugins()

        p.stepSimulation()

        return self.observe()

    def observe(self):
        return {k: v for k,v in {name: receptor.get_observations() for name, receptor in self.receptors.items()}.items() if len(v)}

    def reward(self):
        ret = {k: v for k,v in {name: receptor.get_rewards() for name, receptor in self.receptors.items()}.items() if len(v)}
        return self.sum_dict(ret) if self.sum_rewards else ret

    def is_terminal(self):
        ret = {k: v for k,v in {name: receptor.get_is_terminals() for name, receptor in self.receptors.items()}.items() if len(v)}
        return self.sum_dict(ret) > 0 if self.sum_terminals else ret

    def step(self, action):
        for receptor_name, receptor_action in action.items():
            self.receptors[receptor_name].update_plugins(receptor_action)

        for _ in range(self.sub_steps):
            p.stepSimulation()

        return self.observe(), self.reward(), self.is_terminal(), {}

    def close(self):
        p.disconnect()

    def sum_dict(self, d):
        return sum(self.sum_dict(e) if isinstance(e, dict) else e for e in d.values())


if __name__ == '__main__':

    env = RoboGym('/home/tom/repos/robo-gym/robo_gym/data/environments/ur_high_5.xml')
    a = env.action_space.sample()

    a['ur5_l']['position_controller']['orientation'] = np.array([0.0, 0.0, 0.0])
    a['ur5_l']['position_controller']['position'] = np.array([0.01, 0.0, 0.0])
    a['ur5_r']['position_controller']['orientation'] = np.array([0.0, 0.0, 0.0])
    a['ur5_r']['position_controller']['position'] = np.array([-0.01, 0.0, 0.0])

    for _ in range(10000):
        obs, rew, term, info = env.step(a)

        if term:
            env.reset()
