import numpy as np
import pybullet as p
from gym import spaces
from ..addon import Addon
from robo_gym.model import Model


class DrawCoords(Addon):
    def __init__(self, parent, config):
        super(DrawCoords, self).__init__(parent, config)

        # define the space for the actions this addon expects to receive
        self.action_space = spaces.Box(-1.0, 1.0, shape=(0,), dtype='float32')

        xyz = config.get('xyz', [0.,0.,0.])
        rpy = config.get('rpy', [0.,0.,0.])
        self.scale = config.get('scale', 0.1)
        self.T_offset = self.trans_from_xyz_quat(xyz, p.getQuaternionFromEuler(rpy))

        self.lines = [p.addUserDebugLine([0,0,0], vec, vec) for vec in np.eye(3)]
        self.uid = parent.uid if isinstance(parent, Model) else -1
        self.frame_id = [p.getJointInfo(self.uid, i)[1].decode('utf-8') for i in range(p.getNumJoints(self.uid))].index(config.get('frame')) if config.has_key('frame') else -1

    def update(self, action):
        if self.frame_id >= 0:
            link_state = p.getLinkState(self.uid, self.frame_id)
            xyz, rot = link_state[4], link_state[5]
        elif self.uid >= 0:
            model_state = p.getBasePositionAndOrientation(self.uid)
            xyz, rot = model_state[0], model_state[1]
        else:
            xyz, rot = [0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]

        T_world_parent = self.trans_from_xyz_quat(xyz, rot)
        T_net = T_world_parent.dot(self.T_offset)

        self.lines = [p.addUserDebugLine(T_net[:3,3], T_net[:3,3]+T_net[:3,:3].dot(vec)*self.scale, vec, lineWidth=2, replaceItemUniqueId=line) for vec, line in zip(np.eye(3), self.lines)]

    def trans_from_xyz_quat(self, xyz, quat):
        T = np.eye(4)
        T[:3,3] = xyz
        T[:3,:3] = np.array(p.getMatrixFromQuaternion(quat)).reshape(3,3)
        return T