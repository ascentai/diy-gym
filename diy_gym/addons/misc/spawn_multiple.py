from diy_gym.addons.addon import Addon
from diy_gym.model import Model


class SpawnMultiple(Addon):
    def __init__(self, parent, config):
        super(SpawnMultiple, self).__init__(parent, config)

        child_config = config.find('model')
        parent.models.update({child_config.name + '_%d' % i: Model(child_config) for i in range(config.get('num_models'))})
