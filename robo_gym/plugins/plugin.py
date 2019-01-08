from gym import spaces


class Plugin:
    @staticmethod
    def factory(plugin_type, parent, config):
        from plugins.camera import Camera
        from plugins.position_controller import PositionController
        from plugins.joint_controller import JointController
        from plugins.random_respawn import RandomRespawn
        from plugins.joint_state_sensor import JointStateSensor
        from plugins.end_effector_state_sensor import EndEffectorStateSensor
        from plugins.episode_timer import EpisodeTimer
        from plugins.spawn_multiple import SpawnMultiple

        plugins = {
            'camera': Camera,
            'position_controller': PositionController,
            'joint_controller': JointController,
            'random_respawn': RandomRespawn,
            'joint_state_sensor': JointStateSensor,
            'end_effector_state_sensor': EndEffectorStateSensor,
            'episode_timer': EpisodeTimer,
            'spawn_multiple': SpawnMultiple,
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

    def reward(self, observation):
        pass

    def is_terminal(self, observation):
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

    def get_is_terminals(self, observation):
        return {k: v for k, v in {name: plugin.is_terminal(observation) for name, plugin in self.plugins.items()}.items() if v is not None}

    def get_observations(self):
        return {k: v for k, v in {name: plugin.observe() for name, plugin in self.plugins.items()}.items() if v is not None}

    def get_rewards(self, observation):
        return {k: v for k, v in {name: plugin.reward(observation) for name, plugin in self.plugins.items()}.items() if v is not None}

    def update_plugins(self, action):
        for name, action in action.items():
            self.plugins[name].update(action)
