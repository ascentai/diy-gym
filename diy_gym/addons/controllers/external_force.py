import pybullet as p
from gym import spaces
from diy_gym.addons.addon import Addon


class ExternalForce(Addon):
    """ExternalForce defines a addon that can be used to apply an external force to the base a model.
    """
    def __init__(self, parent, config):
        super(ExternalForce, self).__init__(parent, config)

        # store the uid of the parent model
        self.uid = parent.uid

        # get the config for the position of the force or default to [0 0 0] if it's not specified
        self.xyz = config.get('xyz', [0.0, 0.0, 0.0])

        # define the space for the actions this addon expects to receive
        self.action_space = spaces.Box(-10.0, 10.0, shape=(3, ), dtype='float32')

    def update(self, action):
        """Call the pybullet function to apply the desired for to this model.
        """
        p.applyExternalForce(self.uid, -1, action, self.xyz, p.WORLD_FRAME)
