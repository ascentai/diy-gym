import gym
from gym import spaces
from gym.utils import seeding
from collections import OrderedDict
import pybullet as p
import numpy as np

from .config import Configuration
from .model import Model
from .addons.addon import AddonFactory, Addon, Receptor
from .utils import walk_dict, get_bounds_for_space, get_desc_for_space, flatten, unflatten


class DIYGym(gym.Env, Receptor):
    """DIYGym: A flexible simulation environment for robot arms to drive RL research

    The DIYGym class accepts an yaml config file and uses this to:
    - Spawn a bunch of models in pybullet based on URDF files
    - Wrangle actions and observations / rewards in and out of the addons attached to each of those models
    - Maintain and update its own set of addons which act on the environment itself

    The net result is an OpenAI gym interface that can be stepped, reset, observed etc. All the actual logic that defines what happens
    when these functions are called are defined the addons which in turn are defined by the config file. This keeps the environment nice
    and flexible and the addons nice and reuseable. As a result though, the interesting and task-relevant logic is contained in the addons
    so this file is pretty boring.

    Args:
        config_file (string): A file path pointing to the configuration file describing the environment

    Configs:
        max_episode_steps (int, optional): the number of timesteps to allow per episode (disabled by default)
        hot_start (bool, optional): number of simulation steps to take after a reset before beginning an episode,
            this gives models that are floating at t=0 time to come to rest before the episode begins (default is 1)
        render (bool, optional): whether to render the simulation in a GUI (default is True)
        camera_distance (int, optional): distance of the viewport of the GUI from the target position (defined below)
        camera_pitch (int, optional): the pitch of viewport of the GUI
        camera_target_position (list of ints, optional): a point in world coordinates that will be in the center of
            the GUI viewport
        timestep (float, optional, 1/240.): length of wall time per simulation timestep (default is 1/240s)
        solver_iterations (int, optional, 150): number of iterations of the solver per timestep
            (it's best not to change this without a good reason)
        update_freq (float, optional, 100.): frequency of the updates made by DIY Gym through calls to step()
        gravity (list of floats, optional, [0,0,-9.81]): gravity vector used in simulation
        sum_rewards (bool, optional, False): collapses the rewards dictionary into a single float by summing each element
        terminal_if_all (bool, optional, False): collapses the terminal dictionary into a single bool by ANDing each element
        terminal_if_any (bool, optional, False): collapses the terminal dictionary into a single bool by ORing each element
        flatten_observations (bool, optional, False): collapses the observations dictionary into a numpy array by
            concatenating each element
        flatten_actions (bool, optional, False): if True, the step function will expect a single numpy array containing
            the same number of actions as specified in the environment's action space (default is False)
    """
    def __init__(self, config_file):
        gym.Env.__init__(self)
        Receptor.__init__(self)

        config = Configuration.from_file(config_file)

        self.name = config.name
        self._max_episode_steps = config.get('max_episode_steps') if 'max_episode_steps' in config else None
        self.hot_start = config.get('hot_start', 1)

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

        timestep = config.get('timestep', 1 / 240.)
        sub_steps = int(1. / config.get('update_freq', 100) / timestep)
        iterations = config.get('solver_iterations', 150)
        p.setPhysicsEngineParameter(numSolverIterations=iterations, numSubSteps=sub_steps, fixedTimeStep=timestep)

        gravity = config.get('gravity', [0.0, 0.0, -9.81])
        p.setGravity(gravity[0], gravity[1], gravity[2])

        self.models = OrderedDict(
            sorted({child.name: Model(child)
                    for child in config.find_all('model')}.items(), key=lambda t: t[0]))
        self.addons = OrderedDict(
            sorted(
                {child.name: AddonFactory.build(child.get('addon'), self, child)
                 for child in config.find_all('addon')}.items(),
                key=lambda t: t[0]))
        self.receptors = OrderedDict(sorted({**self.models, self.name: self}.items(), key=lambda t: t[0]))

        self.collapse_rewards_func = sum if config.get('sum_rewards', False) else None
        self.collapse_terminals_func = any if config.get(
            'terminal_if_any', False) else all if config.get('terminal_if_all', False) else None
        self.flatten_observations = config.get('flatten_observations', False)
        self.flatten_actions = config.get('flatten_actions', False)

        self.seed()
        self.reset()

        self.observation_space, self.action_space = spaces.Dict({}), spaces.Dict({})

        for name, receptor in self.receptors.items():
            obs_space, act_space = receptor.build_spaces()

            if len(obs_space.spaces):
                self.observation_space.spaces[name] = obs_space

            if len(act_space.spaces):
                self.action_space.spaces[name] = act_space

        if self.flatten_observations:
            lows, highs = [flatten(get_bounds_for_space(self.observation_space, opt)) for opt in [True, False]]
            self.original_observation_space = self.observation_space
            self.observation_space = gym.spaces.Box(low=lows, high=highs)

        if self.flatten_actions:
            lows, highs = (flatten(get_bounds_for_space(self.action_space, opt)) for opt in [True, False])
            self.original_action_space = self.action_space
            self.action_space = gym.spaces.Box(low=lows, high=highs)

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
            for addon in receptor.addons.values():
                addon.reset()

        for _ in range(self.hot_start):
            p.stepSimulation()

        return self.observe()

    def observe(self):
        """Calls get_observations on each addon attached to either the environment or one of its models.

        Returns:
            dict: A dictionary containing a set of observations collected from each addon
        """
        ret = self.walk_addons(lambda addon: addon.observe())

        return flatten(ret) if self.flatten_observations else ret

    def reward(self):
        """Calls get_rewards on each addon attached to either the environment or one of its models.

        Returns:
            dict: A dictionary containing a set of rewards collected from each addon OR the sum of those rewards if the sum_rewards config is enabled
        """
        ret = self.walk_addons(lambda addon: addon.reward())

        return walk_dict(ret, self.collapse_rewards_func) if self.collapse_rewards_func is not None else ret

    def is_terminal(self):
        """Calls get_is_terminals on each addon attached to either the environment or one of its models.

        Returns:
            dict: A dictionary containing a set of terminals collected from each addon OR the logical sum of those terminals
            if either the terminal_if_any or terminal_if_all configs are enabled
        """
        """ CXXAAHT """  # Ayana's first docstring
        ret = self.walk_addons(lambda addon: addon.is_terminal())

        if self._max_episode_steps is not None:
            if not self.name in ret:
                ret[self.name] = {}
            ret[self.name]['episode_timer'] = self.step_counter >= self._max_episode_steps

        return walk_dict(ret, self.collapse_terminals_func) if self.collapse_terminals_func is not None else ret

    def step(self, action):
        """Calls update on each addon attached to either the environment or one of its models.

        Args:
            action (dict): A dictionary containing actions organised according to the action space defined by the environment

        Returns:
            dict: updated observations of the environment
            dict/float: updated set of rewards
            dict/bool: updated set of flags indicating whether the episode is complete
            dict: an auxiliary info dictionary (not used in DIYGym)
        """
        if self.flatten_actions:
            action = unflatten(action, self.original_action_space)

        for receptor_name, receptor_action in action.items():
            for addon_name, addon_action in receptor_action.items():
                self.receptors[receptor_name].addons[addon_name].update(addon_action)

        self.step_counter += 1
        p.stepSimulation()

        return self.observe(), self.reward(), self.is_terminal(), {}

    def walk_addons(self, func):
        ret = OrderedDict()
        for receptor_name, receptor in self.receptors.items():
            receptor_ret = OrderedDict()
            for addon_name, addon in receptor.addons.items():
                addon_ret = func(addon)
                if addon_ret is not None:
                    receptor_ret[addon_name] = addon_ret
            if len(receptor_ret):
                ret[receptor_name] = receptor_ret

        return ret

    def close(self):
        p.disconnect()
