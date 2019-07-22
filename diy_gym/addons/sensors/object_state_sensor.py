import pybullet as p
import numpy as np
from gym import spaces
from diy_gym.addons.addon import Addon


class ObjectStateSensor(Addon):
    def __init__(self, parent, config):
        super(ObjectStateSensor, self).__init__(parent, config)

        self.source_model = parent.models[config.get('source_model')] if 'source_model' in config else None
        self.target_model = parent.models[config.get('target_model')] if 'target_model' in config else parent

        self.source_frame_id = self.source_model.get_frame_id(
            config.get('source_frame')) if 'source_frame' in config else -1
        self.target_frame_id = self.source_model.get_frame_id(
            config.get('target_frame')) if 'target_frame' in config else -1

        self.include_rotation = config.get('include_rotation', False)
        self.include_velocity = config.get('include_velocity', False)

        self.observation_space = spaces.Dict({'position': spaces.Box(-10, 10, shape=(3, ), dtype='float32')})

        if self.include_rotation:
            self.observation_space.spaces['rotation'] = spaces.Box(-10, 10, shape=(3, ), dtype='float32')

        if self.include_velocity:
            self.observation_space.spaces['velocity'] = spaces.Box(-10, 10, shape=(3, ), dtype='float32')

        if self.include_rotation and self.include_velocity:
            self.observation_space.spaces['angular_velocity'] = spaces.Box(-10, 10, shape=(3, ), dtype='float32')

    def get_state(self, uid, frame_id):
        if frame_id < 0:
            pose = p.getBasePositionAndOrientation(uid)
            twist = p.getBaseVelocity(uid)
            position = pose[0]
            rotation = pose[1]
            velocity = twist[0]
            angular_velocity = twist[1]
        else:
            state = p.getLinkState(uid, frame_id, computeLinkVelocity=1)
            position = state[0]
            rotation = state[1]
            velocity = state[6]
            angular_velocity = state[7]

        return list(np.array(obj) for obj in [position, velocity, rotation, angular_velocity])

    def observe(self):
        obs = {}

        state = self.get_state(self.target_model.uid, self.target_frame_id)

        # if a reference frame / object has been specified then subtract that away from the observation
        if self.source_model is not None:
            source_state = self.get_state(self.source_model.uid, self.source_frame_id)

            state[0] -= source_state[0]
            state[1] -= source_state[1]
            state[2] = self.quaternion_multiply(source_state[2], state[2])
            state[3] -= source_state[3]

        obs['position'] = state[0]

        if self.include_velocity:
            obs['velocity'] = state[1]

        if self.include_rotation:
            obs['rotation'] = p.getEulerFromQuaternion(state[2])

        if self.include_rotation and self.include_velocity:
            obs['angular_velocity'] = state[3]

        return obs

    def quaternion_multiply(self, q1, q0):
        return [
            q1[0] * q0[3] + q1[1] * q0[2] - q1[2] * q0[1] + q1[3] * q0[0],
            -q1[0] * q0[2] + q1[1] * q0[3] + q1[2] * q0[0] + q1[3] * q0[1],
            q1[0] * q0[1] - q1[1] * q0[0] + q1[2] * q0[3] + q1[3] * q0[2],
            -q1[0] * q0[0] - q1[1] * q0[1] - q1[2] * q0[2] + q1[3] * q0[3]
        ]
