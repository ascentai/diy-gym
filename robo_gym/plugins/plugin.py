from gym import spaces


class PluginFactory:
    """PluginFactory is a singeleton factory class (fancy!) for creating plugins from the config file based on a string name.

    The factory pattern in Python is a bit error prone since it usually winds up in circular imports. Using the singleton pattern
    here defers those imports until later which breaks the cycle and also simplifies the process of adding new plugins by the user.
    """
    instance = None

    class __PluginFactory:
        """Inner class for the singleton pattern, courtesy of this reference:
        https://python-3-patterns-idioms-test.readthedocs.io/en/latest/Singleton.html
        """
        def __init__(self):
            from plugins.controllers.position_controller import PositionController
            from plugins.controllers.joint_controller import JointController
            from plugins.sensors.camera import Camera
            from plugins.sensors.joint_state_sensor import JointStateSensor
            from plugins.sensors.end_effector_state_sensor import EndEffectorStateSensor
            from plugins.rewards.reach_target import ReachTarget
            from plugins.rewards.stuck_joint_cost import StuckJointCost
            from plugins.rewards.electricity_cost import ElectricityCost
            from plugins.misc.random_respawn import RandomRespawn
            from plugins.misc.episode_timer import EpisodeTimer
            from plugins.misc.spawn_multiple import SpawnMultiple

            self.plugins = {
                'camera': Camera,
                'position_controller': PositionController,
                'joint_controller': JointController,
                'random_respawn': RandomRespawn,
                'joint_state_sensor': JointStateSensor,
                'end_effector_state_sensor': EndEffectorStateSensor,
                'episode_timer': EpisodeTimer,
                'spawn_multiple': SpawnMultiple,
                'electricity_cost': ElectricityCost,
                'stuck_joint_cost': StuckJointCost,
                'reach_target': ReachTarget
            }

    @staticmethod
    def get():
        """Gets the underlying instance of the factory.
        """
        if not PluginFactory.instance:
            PluginFactory.instance = PluginFactory.__PluginFactory()

        return PluginFactory.instance

    @staticmethod
    def build(name, parent, config):
        """Builds a plugin, passing in a reference to its parent and its section of the config file.

        Args:
            name (str): The name of the plugin (as specified as a key in the plugins dictionary above)
            parent (Receptor): An object of a subclass of Receptor to which the plugin will be attached
            config (config.Configuration): A configuration object containing parameters for the plugin

        Returns:
            Plugin: The newly created plugin
        """
        return PluginFactory.get().plugins[name](parent, config)

    @staticmethod
    def add_plugin(name, cls):
        PluginFactory.get().plugins[name] = cls


class Plugin:
    """Plugins are the workhorses of the environment, they can be attached to a Model or the environment itself and are expected to
    interact with their parents in some way to control or observe the learning task.

    To do that, plugins should subclass Plugin and implement any of the methods declared below (they can also implement none of them, that's fine too). Each of these
    functions are basically hooks called by RoboGym class, see the individual docstrings for more on how to use them.

    Note:
        If a plugin implements either update() or observe() then it's a good idea for them to also define action and observation spaces. The environment will collate
        the action and observation spaces into two spaces.Dict spaces which the client can use to understand what actions are accepted by a plugin and what will be
        returned by observe(). Technically this isn't required for the environment to function... but just do it anyway.
    """
    def __init__(self):
        self.action_space = None
        self.observation_space = None

    def update(self, action):
        """update is executed whenever step() is called on the environment AND the action dictionary passed to step contains an
        element corresponding to this plugin. Plugins implementing this function are expected to update actuator commands or something
        along those lines.

        Args:
            action (sample from gym.spaces): An action corresponding to this plugin supplied by the client (should conform the action_space
            defined in __init__)

        Returns:
            Nothing

        Note:
            If the action dict doesn't contain such an element pertaining to this plugin the update() won't be called,
            so be careful not to forget about your plugins!

            pybullet.stepSimulation() will be executed immediately after all the plugins have updated themselves so don't expect the environment
            to update straight away. Also, probably don't call stepSimulation from your plugin, it'll screw up the other plugins
        """
        pass

    def reset(self):
        """reset is executed whenever reset() is called on the environment. Plugins implementing this function are expected to reset
        their parent to a state from which they are ready to begin a new episode. Plugins implementing this function will probably return
        actuators to their rest positions or reset objects back to their start positions.

        Returns:
            Nothing (unlike in the gym interface, there's no need to return an observation here)

        Note:
            It's up to the client to call reset at their convenience. Even if a plugin reports is_terminal, it's not guranteed that this
            will be followed by an immediate reset().
        """
        pass

    def observe(self):
        """observe is executed at the end of calls to either step() or reset() on the environment. Plugins are expected to gather up sensor readings
        and return them in a form corresponding to self.observation_space defined in __init__.

        Returns:
            sample from gym.spaces: An observation made by the plugin of something pertaining to the state of its parent (should conform the action_space
            defined in __init__)

        Note:
            Again, there's no code that checks to make sure that it does conform, but it's a pain when observations
            don't match the declared space, so don't be a jerk.
        """
        return None

    def reward(self):
        """reward is executed at the end of calls to step() on the environment. Plugins implementing this function should use some state information from
        their parent to generate a signal to indicate the agent's progress on a task.

        Returns:
            float: A scalar reward signal

        Note:
            It's generally a bad idea to reward your agent for something that it can't observe in some way. If your plugin does implement this function
            also consider also implementing observe() to add whatever state information was used to generate the reward into the observation.

            A single environment can have any number of plugins that produce rewards, the environment will collate them all up into a dict and return them from step()
        """
        return None

    def is_terminal(self):
        """is_terminal is executed at the end of calls to step() on the environment. Plugins implementing this function should use some state information from
        their parent to decide whether some task has completed or irrecoverably failed.

        Returns:
            bool: Whether a terminal state has been reached or not

        Note:
            It's up to the client to call reset at their convenience. Even if a plugin reports is_terminal, it's not guranteed that this
            will be followed by an immediate reset().

            A single environment can have any number of plugins that produce is_terminal, the environment will collate them all up into a dict and return them from step()
        """
        return None


class Receptor:
    """The Receptor class encapsulates anything to which a plugin can be attached. Currently this is any model
        spawned in the environment as well as the environment itself.
    """
    def __init__(self):
        self.plugins = {}

    def build_spaces(self):
        """Collates the action and observation spaces declared by each plugin attached to this receptor into a gym.spaces.Dict.
        Action or observation spaces equal to None are ignored and omitted from the dictionary.
        """
        obs_space, act_space = spaces.Dict({}), spaces.Dict({})
        obs_space.spaces = {name: plugin.observation_space for name, plugin in self.plugins.items() if plugin.observation_space is not None}
        act_space.spaces = {name: plugin.action_space for name, plugin in self.plugins.items() if plugin.action_space is not None}
        return obs_space, act_space

    def reset_plugins(self):
        """Calls reset() on each plugin attached to this receptor
        """
        for plugin in self.plugins.values():
            plugin.reset()

    def get_is_terminals(self):
        """Collates the is_terminals reported by each plugin attached to this receptor into a dictionary.
        Is_terminals equal to None are ignored and omitted from the dictionary.

        Returns:
            dict: dict containing is_terminals (bool) from each plugin (excluding any None is_terminals)
        """
        return {k: v for k, v in {name: plugin.is_terminal() for name, plugin in self.plugins.items()}.items() if v is not None}

    def get_observations(self):
        """Collates the observations reported by each plugin attached to this receptor into a dictionary.
        Observations equal to None are ignored and omitted from the dictionary.

        Returns:
            dict: dict containing observations from each plugin (excluding any None observations)
        """
        return {k: v for k, v in {name: plugin.observe() for name, plugin in self.plugins.items()}.items() if v is not None}

    def get_rewards(self):
        """Collates the rewards reported by each plugin attached to this receptor into a dictionary.
        Rewards equal to None are ignored and omitted from the dictionary.

        Returns:
            dict: dict containing rewards (float) from each plugin (excluding any None rewards)
        """
        return {k: v for k, v in {name: plugin.reward() for name, plugin in self.plugins.items()}.items() if v is not None}

    def update_plugins(self, action):
        """Calls update() on each plugin whose name corresponds to one of the names listed in the action dictionary

            Args:
                dict: dict containing actions keyed against the name of the plugin for which that action is intended
        """
        for name, action in action.items():
            self.plugins[name].update(action)
