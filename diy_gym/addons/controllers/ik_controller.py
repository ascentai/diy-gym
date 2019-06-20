import numpy as np
import pybullet as p
from gym import spaces
from diy_gym.addons.addon import Addon


class InverseKinematicsController(Addon):

    def __init__(self, parent, config):
        super(InverseKinematicsController, self).__init__(parent, config)

        self.uid = parent.uid

        joint_info = [p.getJointInfo(self.uid, i) for i in range(p.getNumJoints(self.uid))]
        self.end_effector_joint_id = [info[1].decode('UTF-8') for info in joint_info].index(config.get('end_effector'))
        joints = [info[1].decode('UTF-8') for info in joint_info if info[0] <= self.end_effector_joint_id]

        self.joint_ids = [info[0] for info in joint_info if info[1].decode('UTF-8') in joints and info[3] > -1]

        self.joint_position_lower_limit = [p.getJointInfo(self.uid, joint_id)[8] for joint_id in self.joint_ids]
        self.joint_position_upper_limit = [p.getJointInfo(self.uid, joint_id)[9] for joint_id in self.joint_ids]
        self.torque_limit = [p.getJointInfo(self.uid, joint_id)[10] for joint_id in self.joint_ids]
        self.rest_position = config.get('rest_position', [0] * len(self.joint_ids))

        self.action_space = spaces.Dict({'linear': spaces.Box(-0.01, 0.01, shape=(3,), dtype='float32')})
        self.use_orientation = config.get('use_orientation', False)

        if self.use_orientation:
            self.action_space.spaces['rotation'] = spaces.Box(-0.01, 0.01, shape=(3,), dtype='float32')

        self.reset()

    def reset(self):
        for joint_id, angle in zip(self.joint_ids, self.rest_position):
            p.resetJointState(self.uid, joint_id, angle)

        # self.target_state = [np.array(s) for s in p.getLinkState(self.uid, self.end_effector_joint_id)]

    def update(self, action):
        self.target_state = [np.array(s) for s in p.getLinkState(self.uid, self.end_effector_joint_id)]

        self.target_state[0] += action['linear']

        kwargs = {}
        if self.use_orientation:
            self.target_state[1] = self.quaternion_multiply(self.target_state[1], p.getQuaternionFromEuler(action['rotation']))
            kwargs['targetOrientation'] = self.target_state[1]

        joint_cmds = p.calculateInverseKinematics(
            bodyUniqueId=self.uid,
            endEffectorLinkIndex=self.end_effector_joint_id,
            targetPosition=self.target_state[0],
            lowerLimits=self.joint_position_lower_limit,
            upperLimits=self.joint_position_upper_limit,
            jointRanges=np.subtract(self.joint_position_upper_limit, self.joint_position_lower_limit).tolist(),
            restPoses=self.rest_position,
            **kwargs
        )[:self.end_effector_joint_id - 1]

        p.setJointMotorControlArray(
            self.uid,
            self.joint_ids,
            p.POSITION_CONTROL,
            targetPositions=joint_cmds,
            targetVelocities=[0.0] * len(joint_cmds),
            forces=self.torque_limit,
            positionGains=[0.015] * len(joint_cmds),
            velocityGains=[1.0] * len(joint_cmds),
        )

    def quaternion_multiply(self, q1, q0):
        """Return multiplication of two quaternions."""
        return np.array((
            q1[0]*q0[3] + q1[1]*q0[2] - q1[2]*q0[1] + q1[3]*q0[0],
            -q1[0]*q0[2] + q1[1]*q0[3] + q1[2]*q0[0] + q1[3]*q0[1],
            q1[0]*q0[1] - q1[1]*q0[0] + q1[2]*q0[3] + q1[3]*q0[2],
            -q1[0]*q0[0] - q1[1]*q0[1] - q1[2]*q0[2] + q1[3]*q0[3]), dtype=np.float64)
