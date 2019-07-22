import pybullet as p
import pybullet_data
import os
from gym import spaces
from collections import OrderedDict

from .addons.addon import AddonFactory, Receptor

urdf_path = ['', os.path.join(os.path.dirname(os.path.abspath(__file__)), 'data/'), pybullet_data.getDataPath()]


class Model(Receptor):
    def __init__(self, config, parent=None):
        Receptor.__init__(self)

        self.name = config.name

        self.position = config.get('xyz', [0., 0., 0.])
        self.orientation = p.getQuaternionFromEuler(config.get('rpy', [0., 0., 0.]))
        use_fixed_base = config.get('use_fixed_base', False)
        scale = config.get('scale', 1.0)
        urdf = config.get('model')

        try:
            full_urdf_path = next(
                os.path.join(path, urdf) for path in urdf_path if os.path.isfile(os.path.join(path, urdf)))
        except StopIteration:
            raise ValueError('Could not find URDF: ' + urdf)

        self.uid = p.loadURDF(full_urdf_path, useFixedBase=use_fixed_base, globalScaling=scale)

        if parent is None:
            p.resetBasePositionAndOrientation(self.uid, self.position, self.orientation)
        else:
            parent_frame_id = self.get_frame_id(config.get('parent_frame')) if 'parent_frame' in config else -1
            child_frame_id = self.get_frame_id(config.get('child_frame')) if 'child_frame' in config else -1

            pose = p.getLinkState(parent.uid,
                                  parent_frame_id)[:2] if parent_frame_id != -1 else p.getBasePositionAndOrientation(
                                      parent.uid)[:2]
            p.resetBasePositionAndOrientation(self.uid, *pose)

            p.createConstraint(parent.uid, parent_frame_id, self.uid, child_frame_id, p.JOINT_FIXED, [0, 0, 1],
                               self.position, [0, 0, 0], self.orientation, [0, 0, 0, 1])

        if 'mass' in config:
            p.changeDynamics(self.uid, -1, mass=config.get('mass'))

        if 'color' in config:
            p.changeVisualShape(self.uid, -1, rgbaColor=config.get('color'))

        self.addons = OrderedDict(
            sorted(
                {child.name: AddonFactory.build(child.get('addon'), self, child)
                 for child in config.find_all('addon')}.items(),
                key=lambda t: t[0]))
        self.models = OrderedDict(
            sorted({child.name: Model(child, self)
                    for child in config.find_all('model')}.items(), key=lambda t: t[0]))

    def get_frame_id(self, frame):
        frames = [p.getJointInfo(self.uid, i)[1].decode('utf-8') for i in range(p.getNumJoints(self.uid))]
        return frames.index(frame) if frame in frames else -1

    def get_transform(self, frame_id=-1):
        if frame_id >= 0:
            link_state = p.getLinkState(self.uid, frame_id)
            xyz, rot = link_state[4], link_state[5]
        else:
            model_state = p.getBasePositionAndOrientation(self.uid)
            xyz, rot = model_state[0], model_state[1]

        return xyz, rot
