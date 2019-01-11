import numpy as np
import pybullet as p
from gym import spaces
from robo_gym import RoboGym
from robo_gym.plugins.plugin import Plugin, PluginFactory


class ExternalForce(Plugin):
    """ExternalForce defines a plugin that can be used to apply an external force to the base a model.
    It's used here to show how to define a custom plugin and integrate it into RoboGym.
    """
    def __init__(self, parent, config):
        super(ExternalForce, self).__init__()

        # store the uid of the parent model
        self.uid = parent.uid

        # get the config for teh position of the force or default to [0 0 0] if it's not specified
        self.position = config.get('force_position', [0.0, 0.0, 0.0])

        # define the space for the actions this plugin expects to receive
        self.action_space = spaces.Box(-10.0, 10.0, shape=(3,), dtype='float32')

    def update(self, action):
        """Call the pybullet function to apply the desired for to this model.
        """
        p.applyExternalForce(self.uid, -1, action, self.position, p.WORLD_FRAME)


if __name__ == '__main__':
    PluginFactory.add_plugin('external_force', ExternalForce)

    env = RoboGym('jaco_on_a_table.xml')

    action = env.action_space.sample()

    action['robot']['controller']['position'] = np.array([0.005, 0.005, -0.005])
    action['robot']['controller']['orientation'] = np.array([-0.02, 0.02, -0.02])
    action['purple_thing']['external_force'] = np.array([1.0, 0.0, 0.0])

    while True:
        observation, reward, terminal, info = env.step(action)
