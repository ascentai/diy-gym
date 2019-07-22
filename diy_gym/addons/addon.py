from gym import spaces
from collections import OrderedDict


class AddonFactory:
    """AddonFactory is a singeleton factory class (fancy!) for creating addons from the config file based on a string name.

    The factory pattern in Python is a bit error prone since it usually winds up in circular imports. Using the singleton pattern
    here defers those imports until later which breaks the cycle and also simplifies the process of adding new addons by the user.
    """
    instance = None

    class __AddonFactory:
        """Inner class for the singleton pattern, courtesy of this reference:
        https://python-3-patterns-idioms-test.readthedocs.io/en/latest/Singleton.html
        """
        def __init__(self):
            from .controllers.ik_controller import InverseKinematicsController
            from .controllers.joint_controller import JointController
            from .controllers.external_force import ExternalForce
            from .sensors.camera import Camera
            from .sensors.joint_state_sensor import JointStateSensor
            from .sensors.object_state_sensor import ObjectStateSensor
            from .sensors.force_torque_sensor import ForceTorqueSensor
            from .rewards.reach_target import ReachTarget
            from .rewards.stuck_joint_cost import StuckJointCost
            from .rewards.electricity_cost import ElectricityCost
            from .rewards.time_penalty import TimePenalty
            from .misc.respawn import Respawn
            from .misc.spawn_multiple import SpawnMultiple
            from .misc.draw_coords import DrawCoords
            from .misc.visual_randomizer import VisualRandomizer
            from .misc.dynamics_randomizer import DynamicsRandomizer

            self.addons = {
                'ik_controller': InverseKinematicsController,
                'joint_controller': JointController,
                'camera': Camera,
                'joint_state_sensor': JointStateSensor,
                'object_state_sensor': ObjectStateSensor,
                'force_torque_sensor': ForceTorqueSensor,
                'reach_target': ReachTarget,
                'stuck_joint_cost': StuckJointCost,
                'electricity_cost': ElectricityCost,
                'time_penalty': TimePenalty,
                'respawn': Respawn,
                'spawn_multiple': SpawnMultiple,
                'draw_coords': DrawCoords,
                'external_force': ExternalForce,
                'visual_randomizer': VisualRandomizer,
                'dynamics_randomizer': DynamicsRandomizer,
            }

    @staticmethod
    def get():
        """Gets the underlying instance of the factory.
        """
        if not AddonFactory.instance:
            AddonFactory.instance = AddonFactory.__AddonFactory()

        return AddonFactory.instance

    @staticmethod
    def build(name, parent, config):
        """Builds a addon, passing in a reference to its parent and its section of the config file.

        Args:
            name (str): The name of the addon (as specified as a key in the addons dictionary above)
            parent (Receptor): An object of a subclass of Receptor to which the addon will be attached
            config (config.Configuration): A configuration object containing parameters for the addon

        Returns:
            Addon: The newly created addon
        """
        return AddonFactory.get().addons[name](parent, config)

    @staticmethod
    def register_addon(name, cls):
        AddonFactory.get().addons[name] = cls


class _Ignore(object):
    pass


_ignore = _Ignore()


class Addon:
    """Addons are the workhorses of the environment, they can be attached to a Model or the environment itself and are expected to
    interact with their parents in some way to control or observe the learning task.

    To do that, addons should subclass Addon and implement any of the methods declared below (they can also implement none of them, that's fine too). Each of these
    functions are basically hooks called by DIYGym class, see the individual docstrings for more on how to use them.

    Note:
        If a addon implements either update() or observe() then it's a good idea for them to also define action and observation spaces. The environment will collate
        the action and observation spaces into two spaces.Dict spaces which the client can use to understand what actions are accepted by a addon and what will be
        returned by observe(). Technically this isn't required for the environment to function... but just do it anyway.
    """
    def __init__(self, parent, config):
        self.parent = parent
        self.action_space = None
        self.observation_space = None
        self.hide = config.get('hide', False)

    def update(self, action):
        """update is executed whenever step() is called on the environment AND the action dictionary passed to step contains an
        element corresponding to this addon. Addons implementing this function are expected to update actuator commands or something
        along those lines.

        Args:
            action (sample from gym.spaces): An action corresponding to this addon supplied by the client (should conform the action_space
            defined in __init__)

        Returns:
            Nothing

        Note:
            If the action dict doesn't contain such an element pertaining to this addon the update() won't be called,
            so be careful not to forget about your addons!

            pybullet.stepSimulation() will be executed immediately after all the addons have updated themselves so don't expect the environment
            to update straight away. Also, probably don't call stepSimulation from your addon, it'll screw up the other addons
        """
        pass  #return _ignore

    def reset(self):
        """reset is executed whenever reset() is called on the environment. Addons implementing this function are expected to reset
        their parent to a state from which they are ready to begin a new episode. Addons implementing this function will probably return
        actuators to their rest positions or reset objects back to their start positions.

        Returns:
            Nothing (unlike in the gym interface, there's no need to return an observation here)

        Note:
            It's up to the client to call reset at their convenience. Even if a addon reports is_terminal, it's not guranteed that this
            will be followed by an immediate reset().
        """
        pass  #return _ignore

    def observe(self):
        """observe is executed at the end of calls to either step() or reset() on the environment. Addons are expected to gather up sensor readings
        and return them in a form corresponding to self.observation_space defined in __init__.

        Returns:
            sample from gym.spaces: An observation made by the addon of something pertaining to the state of its parent (should conform the action_space
            defined in __init__)

        Note:
            Again, there's no code that checks to make sure that it does conform, but it's a pain when observations
            don't match the declared space, so don't be a jerk.
        """
        pass  #return _ignore

    def reward(self):
        """reward is executed at the end of calls to step() on the environment. Addons implementing this function should use some state information from
        their parent to generate a signal to indicate the agent's progress on a task.

        Returns:
            float: A scalar reward signal

        Note:
            It's generally a bad idea to reward your agent for something that it can't observe in some way. If your addon does implement this function
            also consider also implementing observe() to add whatever state information was used to generate the reward into the observation.

            A single environment can have any number of addons that produce rewards, the environment will collate them all up into a dict and return them from step()
        """
        pass  #return _ignore

    def is_terminal(self):
        """is_terminal is executed at the end of calls to step() on the environment. Addons implementing this function should use some state information from
        their parent to decide whether some task has completed or irrecoverably failed.

        Returns:
            bool: Whether a terminal state has been reached or not

        Note:
            It's up to the client to call reset at their convenience. Even if a addon reports is_terminal, it's not guranteed that this
            will be followed by an immediate reset().

            A single environment can have any number of addons that produce is_terminal, the environment will collate them all up into a dict and return them from step()
        """
        pass  #return _ignore


class Receptor:
    """The Receptor class encapsulates anything to which a addon can be attached. Currently this is any model
        spawned in the environment as well as the environment itself.
    """
    def __init__(self):
        self.addons = OrderedDict()

    def build_spaces(self):
        """Collates the action and observation spaces declared by each addon attached to this receptor into a gym.spaces.Dict.
        Action or observation spaces equal to None are ignored and omitted from the dictionary.
        """
        obs_space, act_space = spaces.Dict({}), spaces.Dict({})

        for name, addon in self.addons.items():
            if not addon.hide:
                if addon.observation_space is not None:
                    obs_space.spaces[name] = addon.observation_space

                if addon.action_space is not None:
                    act_space.spaces[name] = addon.action_space

        return obs_space, act_space
