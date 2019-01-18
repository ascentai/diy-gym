import gym
from gym import spaces
from gym.utils import seeding
import pybullet as p
import numpy as np

from .config import Configuration
from .model import Model
from .addons.addon import AddonFactory, Receptor


class RoboGym(gym.Env, Receptor):
    """RoboGym: A flexible simulation environment for robot arms to drive RL research @ Ascent

    The RoboGym class accepts an XML config file and uses this to:
    - Spawn a bunch of models in pybullet based on URDF files
    - Wrangle actions and observations / rewards in and out of the addons attached to each of those models
    - Maintain and update its own set of addons which act on the environment itself

    The net result is an OpenAI gym interface that can be stepped, reset, observed etc. All the actual logic that defines what happens
    when these functions are called are defined the addons which in turn are defined by the config file. This keeps the environment nice
    and flexible and the addons nice and reuseable. As a result though, the interesting and task-relevant logic is contained in the addons
    so this file is pretty boring.

    Args:
        config_file (string): A file path pointing to the configuration file describing the environment
    """
    def __init__(self, config_file):
        gym.Env.__init__(self)
        Receptor.__init__(self)

        config = Configuration.from_file(config_file)

        self.sum_rewards = config.get('sum_rewards', False)
        self.sum_terminals = config.get('sum_terminals', False)
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
        self.addons = {child.attributes['name']: AddonFactory.build(child.attributes['type'], self, child) for child in config.find_all('addon')}
        self.receptors = {**self.models, 'environment': self}

        self.observation_space, self.action_space = spaces.Dict({}), spaces.Dict({})

        for name, receptor in self.receptors.items():
            obs_space, act_space = receptor.build_spaces()
            if len(obs_space.spaces): self.observation_space.spaces[name] = obs_space
            if len(act_space.spaces): self.action_space.spaces[name] = act_space

        self.seed()
        self.reset()

    def seed(self, seed=None):
        """Set the random seeds for the environment
        """
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def reset(self):
        """Calls reset() on each addon attached to either the environment or one of its models.
        Exactly what this does (or even if it does anything at all) is up to the addons however
        the general effect should be to restore the environment to its original state ahead of
        running a new episode.

        Returns:
            dict: A dictionary containing a set of observations collected from each addon
        """
        for receptor in self.receptors.values():
            receptor.reset_addons()

        p.stepSimulation()

        return self.observe()

    def observe(self):
        """Calls observe() on each addon attached to either the environment or one of its models.
            Exactly what this does is up to the addons however the general effect should be to restore
            the environment to its original state ahead of running a new episode.
        """
        return {k: v for k,v in {name: receptor.get_observations() for name, receptor in self.receptors.items()}.items() if len(v)}

    def reward(self):
        ret = {k: v for k,v in {name: receptor.get_rewards() for name, receptor in self.receptors.items()}.items() if len(v)}
        return self.walk_dict(ret) if self.sum_rewards else ret

    def is_terminal(self):
        ret = {k: v for k,v in {name: receptor.get_is_terminals() for name, receptor in self.receptors.items()}.items() if len(v)}
        return self.walk_dict(ret) > 0 if self.sum_terminals else ret

    def step(self, action):
        for receptor_name, receptor_action in action.items():
            self.receptors[receptor_name].update_addons(receptor_action)

        for _ in range(self.sub_steps):
            p.stepSimulation()

        return self.observe(), self.reward(), self.is_terminal(), {}

    def close(self):
        p.disconnect()

    def walk_dict(self, d, func=sum):
        return func(self.walk_dict(e) if isinstance(e, dict) else e for e in d.values())


if __name__ == '__main__':

    env = RoboGym('/home/tom/repos/robo-gym/robo_gym/data/environments/jaco_on_a_table.xml')

    a = env.action_space.sample()

    a['robot']['controller']['position'] = np.array([0.005, 0.005, -0.005])
    a['robot']['controller']['orientation'] = np.array([-0.02, 0.02, -0.02])

    # a['ur5_l']['position_controller']['position'] = np.array([0.005, 0.0, 0.0])
    # a['ur5_r']['position_controller']['position'] = np.array([-0.005, 0.0, 0.0])
    # a['ur5_l']['position_controller']['orientation'][:] = 0
    # a['ur5_r']['position_controller']['orientation'][:] = 0

    for _ in range(100000):
        obs, rew, term, info = env.step(a)

        # if term['ur5_l']['slap'] and term['ur5_r']['slap']:
        #     print('slap!')
        #     env.reset()
