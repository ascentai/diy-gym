from gym import spaces


class Plugin:
    @staticmethod
    def factory(plugin_type, parent, config):
        from plugins.controllers.position_controller import PositionController
        from plugins.controllers.joint_controller import JointController
        from plugins.sensors.camera import Camera
        from plugins.sensors.joint_state_sensor import JointStateSensor
        from plugins.sensors.end_effector_state_sensor import EndEffectorStateSensor
        from plugins.rewards.electricity_cost import ElectricityCost
        from plugins.misc.random_respawn import RandomRespawn
        from plugins.misc.episode_timer import EpisodeTimer
        from plugins.misc.spawn_multiple import SpawnMultiple

        plugins = {
            'camera': Camera,
            'position_controller': PositionController,
            'joint_controller': JointController,
            'random_respawn': RandomRespawn,
            'joint_state_sensor': JointStateSensor,
            'end_effector_state_sensor': EndEffectorStateSensor,
            'episode_timer': EpisodeTimer,
            'spawn_multiple': SpawnMultiple,
            'electricity_cost': ElectricityCost
        }

        return plugins[plugin_type](parent, config)

    def __init__(self):
        self.action_space = None
        self.observation_space = None

    def update(self, action):
        pass

    def observe(self):
        pass

    def reset(self):
        pass

    def reward(self):
        pass

    def is_terminal(self):
        pass


class Receptor:
    def __init__(self):
        self.plugins = {}

    def build_spaces(self):
        obs_space, act_space = spaces.Dict({}), spaces.Dict({})
        obs_space.spaces = {name: plugin.observation_space for name, plugin in self.plugins.items() if plugin.observation_space is not None}
        act_space.spaces = {name: plugin.action_space for name, plugin in self.plugins.items() if plugin.action_space is not None}
        return obs_space, act_space

    def reset_plugins(self):
        for plugin in self.plugins.values():
            plugin.reset()

    def get_is_terminals(self):
        return {k: v for k, v in {name: plugin.is_terminal() for name, plugin in self.plugins.items()}.items() if v is not None}

    def get_observations(self):
        return {k: v for k, v in {name: plugin.observe() for name, plugin in self.plugins.items()}.items() if v is not None}

    def get_rewards(self):
        return {k: v for k, v in {name: plugin.reward() for name, plugin in self.plugins.items()}.items() if v is not None}

    def update_plugins(self, action):
        for name, action in action.items():
            self.plugins[name].update(action)
