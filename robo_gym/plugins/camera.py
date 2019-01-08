import pybullet as p
import numpy as np
from gym import spaces
from plugins.plugin import Plugin


def trans_from_xyz_quat(xyz, quat):
    T = np.eye(4)
    T[:3,3] = xyz
    T[:3,:3] = np.array(p.getMatrixFromQuaternion(quat)).reshape(3,3)
    return T

class Camera(Plugin):
    def __init__(self, parent, config):
        super(Camera, self).__init__()

        self.near, self.far = config.get('clipping_boundaries', [0.01, 100])
        self.fov = config.get('field_of_view', 70.0)
        self.resolution = config.get('resolution', [640, 480])
        self.aspect = self.resolution[0] / self.resolution[1]

        self.uid = parent.uid
        self.frame_id = [p.getJointInfo(self.uid, i)[1].decode('utf-8') for i in range(p.getNumJoints(self.uid))].index(config.get('base_frame'))

        xyz = config.get('xyz', [0.,0.,0.])
        rpy = config.get('rpy', [0.,0.,0.])
        rpy[1] += np.pi

        self.T_model_cam = trans_from_xyz_quat(xyz, p.getQuaternionFromEuler(rpy))

        self.projection_matrix = p.computeProjectionMatrixFOV(self.fov, self.aspect, self.near, self.far)
        self.K = np.array(self.projection_matrix).reshape([4, 4]).T

        self.observation_space = spaces.Dict({
            'rgb': spaces.Box(0., 1., shape=self.resolution+[3], dtype='float32'),
            'depth': spaces.Box(0., 10., shape=self.resolution, dtype='float32')
        })

    def observe(self):
        obs = {}

        link_state = p.getLinkState(self.uid, self.frame_id)

        T_world_model = trans_from_xyz_quat(link_state[4], link_state[5])
        T_world_cam = np.linalg.inv(T_world_model.dot(self.T_model_cam))

        image = p.getCameraImage(self.resolution[0], self.resolution[1], T_world_cam.T.flatten(), self.projection_matrix)

        rgb = np.array(image[2]).reshape([self.resolution[1], self.resolution[0], 4])[:, :, :3] / 255.  # discard the alpha channel and normalise to [0 1]

        # the depth buffer is normalised to [0 1] whereas NDC coords require [-1 1] ref: https://bit.ly/2rcXidZ
        depth_ndc = np.array(image[3]) * 2 - 1

        # recover eye coordinate depth using the projection matrix ref: https://bit.ly/2vZJCsx
        depth = self.K[2,3] / (self.K[3,2] * depth_ndc - self.K[2,2])

        obs['rgb'] = rgb
        obs['depth'] = depth

        return obs