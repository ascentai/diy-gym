import pybullet as p
import pybullet_data
import os
from gym import spaces

from .addons.addon import AddonFactory, Receptor


urdf_path = [
    '',
    os.path.join(os.path.dirname(os.path.abspath(__file__)), 'data/'),
    pybullet_data.getDataPath()
]

class Model(Receptor):
    def __init__(self, config):
        Receptor.__init__(self)

        position = config.get('xyz', [0.,0.,0.])
        orientation = p.getQuaternionFromEuler(config.get('rpy', [0.,0.,0.]))
        use_fixed_base = config.get('use_fixed_base', False)
        scale = config.get('scale', 1.0)
        self.urdf = config.get('model')

        try:
            full_urdf_path = next(os.path.join(path, self.urdf) for path in urdf_path if os.path.isfile(os.path.join(path, self.urdf)))
        except StopIteration:
            raise ValueError('Could not find URDF: ' + self.urdf)

        self.uid = p.loadURDF(full_urdf_path, useFixedBase=use_fixed_base, globalScaling=scale)

        p.resetBasePositionAndOrientation(self.uid, position, orientation)

        if config.has_key('mass'):
            p.changeDynamics(self.uid, -1, mass=config.get('mass'))

        if config.has_key('color'):
            p.changeVisualShape(self.uid, -1, rgbaColor=config.get('color'))

        self.addons = {child.name: AddonFactory.build(child.get('addon'), self, child) for child in config.find_all('addon')}
