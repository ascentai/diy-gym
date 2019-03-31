import numpy as np

from ..addon import Addon
from .object_state_sensor import ObjectStateSensor


class DistanceSensor(Addon):

    def __init__(self, parent, config):
        super(DistanceSensor, self).__init__(parent, config)

        self.config = config

        self.object1 = parent.models[config.get('origin')]
        self.object1_link = config.get('origin_link') if config.has_key('origin_link') else None

        self.object2 = parent.models[config.get('target')]
        self.object2_link = config.get('target_link') if config.has_key('target_link') else None

    def observe(self):
        object1_position = ObjectStateSensor(self.object1, self.config, self.object1_link).observe()['position']
        object2_position = ObjectStateSensor(self.object2, self.config, self.object2_link).observe()['position']

        return {'distance': self.calculate_distance(object1_position, object2_position)}

    @staticmethod
    def calculate_distance(a, b):
        return np.sqrt(np.sum(np.power(np.array(a) - np.array(b), 2)))
