import pybullet as p
import numpy as np
from gym import spaces
from diy_gym.model import Model
from diy_gym.addons.addon import Addon


class Camera(Addon):
    def __init__(self, parent, config):
        super(Camera, self).__init__(parent, config)

        self.near, self.far = config.get('clipping_boundaries', [0.01, 100])
        self.fov = config.get('field_of_view', 70.0)
        self.resolution = config.get('resolution', [640, 480])
        self.aspect = self.resolution[0] / self.resolution[1]

        self.uid = parent.uid if isinstance(parent, Model) else -1
        self.frame_id = parent.get_frame_id(config.get('frame')) if 'frame' in config else -1

        xyz = config.get('xyz', [0.,0.,0.])
        rpy = config.get('rpy', [0.,0.,0.])

        self.use_depth = config.get('use_depth', True)
        self.use_seg_mask = config.get('use_segmentation_mask', False)

        self.T_parent_cam = self.trans_from_xyz_quat(xyz, p.getQuaternionFromEuler(rpy))
        self.projection_matrix = p.computeProjectionMatrixFOV(self.fov, self.aspect, self.near, self.far)
        self.K = np.array(self.projection_matrix).reshape([4, 4]).T

        self.observation_space = spaces.Dict({'rgb': spaces.Box(0., 1., shape=self.resolution+[3], dtype='float32'),})

        if self.use_depth:
            self.observation_space.spaces.update({'depth': spaces.Box(0., 10., shape=self.resolution, dtype='float32')})

        if self.use_seg_mask:
            self.observation_space.spaces.update({'segmentation_mask': spaces.Box(0., 10., shape=self.resolution, dtype='float32')})

    def observe(self):
        if self.frame_id >= 0:
            link_state = p.getLinkState(self.uid, self.frame_id)
            T_world_parent = self.trans_from_xyz_quat(link_state[4], link_state[5])
        elif self.uid >= 0:
            model_state = p.getBasePositionAndOrientation(self.uid)
            T_world_parent = self.trans_from_xyz_quat(model_state[0], model_state[1])
        else:
            T_world_parent = np.eye(4)

        T_world_cam = np.linalg.inv(T_world_parent.dot(self.T_parent_cam))

        image = p.getCameraImage(self.resolution[0], self.resolution[1], T_world_cam.T.flatten(), self.projection_matrix, flags=p.ER_NO_SEGMENTATION_MASK if not self.use_seg_mask else 0)

        obs = {'rgb': np.array(image[2], copy=False).reshape(self.resolution+[4])[:, :, :3] / 255.} # discard the alpha channel and normalise to [0 1]

        if self.use_depth:
            # the depth buffer is normalised to [0 1] whereas NDC coords require [-1 1] ref: https://bit.ly/2rcXidZ
            depth_ndc = np.array(image[3], copy=False).reshape(self.resolution) * 2 - 1

            # recover eye coordinate depth using the projection matrix ref: https://bit.ly/2vZJCsx
            depth = self.K[2,3] / (self.K[3,2] * depth_ndc - self.K[2,2])

            obs['depth'] = depth

        if self.use_seg_mask:
            obs['segmentation_mask'] = np.array(image[4], copy=False).reshape(self.resolution)

        return obs

    def trans_from_xyz_quat(self, xyz, quat):
        T = np.eye(4)
        T[:3,3] = xyz
        T[:3,:3] = np.array(p.getMatrixFromQuaternion(quat)).reshape(3,3)
        return T