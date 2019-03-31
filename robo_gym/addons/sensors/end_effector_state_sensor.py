import pybullet as p
import numpy as np
from gym import spaces
from ..addon import Addon


class EndEffectorStateSensor(Addon):

    def __init__(self, parent, config):
        super(EndEffectorStateSensor, self).__init__(parent, config)

        self.uid = parent.uid
        self.ee_id = [p.getJointInfo(self.uid, i)[1].decode('utf-8') for i in
                      range(p.getNumJoints(self.uid))].index(config.get('end_effector_frame'))

        self.observation_space = spaces.Dict({
            'position': spaces.Box(-10, 10, shape=(3,), dtype='float32'),
            # 'orientation': spaces.Box(-10, 10, shape=(3,), dtype='float32'),
            # 'velocity': spaces.Box(-10, 10, shape=(3,), dtype='float32'),
            # 'angular_velocity': spaces.Box(-10, 10, shape=(3,), dtype='float32'),
            # 'force': spaces.Box(-10, 10, shape=(3,), dtype='float32'),
            # 'torque': spaces.Box(-10, 10, shape=(3,), dtype='float32')
        })

    def observe(self):
        link_state = p.getLinkState(self.uid, self.ee_id, computeLinkVelocity=1)

        # joint_states = p.getJointStates(self.uid, self.joint_ids)
        # position = [state[0] for state in joint_states]
        # torque = [state[3] for state in joint_states]

        # g = p.calculateInverseDynamics(self.uid, position, np.zeros_like(position), np.zeros_like(position))
        # J = np.vstack(p.calculateJacobian(self.uid, self.gripper_id, [0., 0., 0.], position, np.zeros_like(position),
        #                                   np.zeros_like(position)))

        # try:
        #     vt = np.linalg.lstsq(J.transpose(), np.array(g) - np.array(torque), rcond=None)[0]
        # except ValueError as e:
        #     print(e)
        #     vt = np.zeros(6, )

        obs = {
            'position': link_state[0],
            # 'velocity': link_state[-2],
            # 'orientation': p.getEulerFromQuaternion(link_state[1]),
            # 'angular_velocity': link_state[-1],
            # 'force': vt[:3],
            # 'torque': vt[3:],
        }

        return obs