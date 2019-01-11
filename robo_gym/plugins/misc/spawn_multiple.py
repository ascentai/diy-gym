from ..plugin import Plugin
from robo_gym.model import Model


class SpawnMultiple(Plugin):
    def __init__(self, parent, config):
        super(SpawnMultiple, self).__init__()

        child_config = config.find('model')
        parent.models.update({child_config.attributes['name'] + '_%d' % i: Model(child_config) for i in range(config.get('num_models'))})
