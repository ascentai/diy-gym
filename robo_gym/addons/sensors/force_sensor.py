import numpy as np
import pybullet as p

from .object_state_sensor import ObjectStateSensor


class ForceSensor(ObjectStateSensor):

    def __init__(self, parent, config):
        super(ForceSensor, self).__init__(parent, config, config.get('link_name'), force_enabled=True)

        self.k = config.get('k', 1)
        self.enable_offset = config.get('enable_offset', False)
        self.axis = config.get('axis') if config.has_key('axis') else None

        self.axes = {'x': 0, 'y': 1, 'z': 2}
        self.offset = 0

        if self.enable_offset:
            self.offset = self.calc_offset(self.link_id)

    def observe(self):
        forces_measured = super(ForceSensor, self).observe()['forces']

        if self.axis is not None:
            return {'force_measured': self.k * (forces_measured[self.axes[self.axis]] + self.offset)}

        return {'forces_measured': self.k * forces_measured}

    def calc_offset(self, link_id, gravity=9.81):
        # mass in kg
        total_mass = 0

        # currently works for UR5 w/ 2F gripper
        for link in range(link_id, p.getNumJoints(self.uid)):
            total_mass += p.getDynamicsInfo(self.uid, link)[0]

        return total_mass * gravity

