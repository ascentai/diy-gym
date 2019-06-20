import numpy as np
import pybullet as p
from gym import spaces
from diy_gym.addons.addon import Addon
from diy_gym.model import Model


class DrawCoords(Addon):
    def __init__(self, parent, config):
        super(DrawCoords, self).__init__(parent, config)

        self.action_space = spaces.Box(-1.0, 1.0, shape=(0, ), dtype='float32')

        xyz = config.get('xyz', [0., 0., 0.])
        rpy = config.get('rpy', [0., 0., 0.])
        self.scale = config.get('scale', 0.1)
        self.annotate = config.get('annotate', True)

        self.T_offset = self.trans_from_xyz_quat(xyz, p.getQuaternionFromEuler(rpy))

        if 'frame' in config:
            frames = [config.get('frame')]
        elif 'frames' in config:
            frames = config.get('frames')
        else:
            frames = [p.getJointInfo(self.parent.uid, i)[1].decode('utf-8') for i in range(p.getNumJoints(self.parent.uid))] + ['base']

        self.markers = {
            frame: [
                -1 if frame == 'base' else self.parent.get_frame_id(frame),
                [p.addUserDebugLine([0, 0, 0], vec, vec) for vec in np.eye(3)],
                p.addUserDebugText(frame, [0, 0, 0])
                if self.annotate else None
            ]
            for frame in frames
        }

    def update(self, action):
        for frame in self.markers:
            xyz, rot = self.parent.get_transform(self.markers[frame][0])

            T_world_parent = self.trans_from_xyz_quat(xyz, rot)
            T_net = self.T_offset.dot(T_world_parent)

            self.markers[frame][1] = [
                p.addUserDebugLine(
                    T_world_parent[:3, 3],
                    T_world_parent[:3, 3] + T_world_parent[:3, :3].dot(vec) * self.scale,
                    vec,
                    lineWidth=2,
                    replaceItemUniqueId=line
                ) for vec, line in zip(np.eye(3), self.markers[frame][1])
            ]

            if self.annotate:
                self.markers[frame][2] = p.addUserDebugText(
                    frame,
                    T_net[:3, 3],
                    replaceItemUniqueId=self.markers[frame][2]
                )

    def trans_from_xyz_quat(self, xyz, quat):
        T = np.eye(4)
        T[:3, 3] = xyz
        T[:3, :3] = np.array(p.getMatrixFromQuaternion(quat)).reshape(3, 3)
        return T