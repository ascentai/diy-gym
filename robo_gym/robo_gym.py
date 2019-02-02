import gym
from gym import spaces
from gym.utils import seeding
import pybullet as p
import numpy as np

from .config import Configuration
from .model import Model
from .addons.addon import AddonFactory, Receptor
from .utils import walk_dict, get_bounds_for_space, get_desc_for_space, flatten, unflatten


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

        self.name = config.name
        self.episode_length = config.get('episode_length') if config.has_key('episode_length') else None
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

        self.models = {child.name: Model(child) for child in config.find_all('model')}
        self.addons = {child.name: AddonFactory.build(child.get('addon'), self, child) for child in config.find_all('addon')}
        self.receptors = {**self.models, self.name: self}

        self.collapse_rewards_func = sum if config.get('sum_rewards', False) else None
        self.collapse_terminals_func = any if config.get('terminal_if_any', False) else all if config.get('terminal_if_all', False) else None
        self.flatten_observations = config.get('flatten_observations', False)
        self.flatten_actions = config.get('flatten_actions', False)

        self.seed()
        self.reset()

        self.observation_space, self.action_space = spaces.Dict({}), spaces.Dict({})

        for name, receptor in self.receptors.items():
            obs_space, act_space = receptor.build_spaces()
            if len(obs_space.spaces): self.observation_space.spaces[name] = obs_space
            if len(act_space.spaces): self.action_space.spaces[name] = act_space

        if self.flatten_observations:
            self.original_observation_space = self.observation_space
            lows = flatten(get_bounds_for_space(self.observation_space, low_not_high=True))
            highs = flatten(get_bounds_for_space(self.observation_space, low_not_high=False))
            self.observation_space = gym.spaces.Box(low=lows, high=highs)
            self.observation_desc = get_desc_for_space(self.original_observation_space)

        if self.flatten_actions:
            self.original_action_space = self.action_space
            lows = flatten(get_bounds_for_space(self.action_space, low_not_high=True))
            highs = flatten(get_bounds_for_space(self.action_space, low_not_high=False))
            self.action_space = gym.spaces.Box(low=lows, high=highs)
            self.action_desc = get_desc_for_space(self.original_action_space)

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
        self.step_counter = 0

        for receptor in self.receptors.values():
            receptor.reset_addons()

        p.stepSimulation()

        return self.observe()

    def observe(self):
        """Calls get_observations on each addon attached to either the environment or one of its models.

        Returns:
            dict: A dictionary containing a set of observations collected from each addon
        """
        ret = {k: v for k,v in {name: receptor.get_observations() for name, receptor in self.receptors.items()}.items() if len(v)}

        return flatten(ret) if self.flatten_observations else ret

    def reward(self):
        """Calls get_rewards on each addon attached to either the environment or one of its models.

        Returns:
            dict: A dictionary containing a set of rewards collected from each addon OR the sum of those rewards if the sum_rewards config is enabled
        """
        ret = {k: v for k,v in {name: receptor.get_rewards() for name, receptor in self.receptors.items()}.items() if len(v)}
        return walk_dict(ret, self.collapse_rewards_func) if self.collapse_rewards_func is not None else ret

    def is_terminal(self):
        """Calls get_is_terminals on each addon attached to either the environment or one of its models.

        Returns:
            dict: A dictionary containing a set of terminals collected from each addon OR the logical sum of those terminals
            if either the terminal_if_any or terminal_if_al configs are enabled
        """
        ret = {k: v for k,v in {name: receptor.get_is_terminals() for name, receptor in self.receptors.items()}.items() if len(v)}

        if self.episode_length is not None:
            if not self.name in ret:
                ret[self.name] = {}

            ret[self.name]['episode_timer'] = self.step_counter >= self.episode_length

        return walk_dict(ret, self.collapse_terminals_func) if self.collapse_terminals_func is not None else ret

    def step(self, action):
        """Calls update on each addon attached to either the environment or one of its models.

        Args:
            action (dict): A dictionary containing actions organised according to the action space defined by the environment

        Returns:
            dict: updated observations of the environment
            dict/float: updated set of rewards
            dict/bool: updated set of flags indicating whether the episode is complete
            dict: an auxiliary info dictionary (not used in RoboGym)
        """
        if self.flatten_actions:
            action = unflatten(action, self.original_action_space)

        for receptor_name, receptor_action in action.items():
            self.receptors[receptor_name].update_addons(receptor_action)

        self.step_counter += 1

        for _ in range(self.sub_steps):
            p.stepSimulation()

        return self.observe(), self.reward(), self.is_terminal(), {}

    def close(self):
        p.disconnect()
