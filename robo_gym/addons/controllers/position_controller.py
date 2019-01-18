import numpy as np
import pybullet as p
from gym import spaces
from ..addon import Addon


def quaternion_multiply(q1, q0):
    """Return multiplication of two quaternions."""
    return np.array((
         q1[0]*q0[3] + q1[1]*q0[2] - q1[2]*q0[1] + q1[3]*q0[0],
        -q1[0]*q0[2] + q1[1]*q0[3] + q1[2]*q0[0] + q1[3]*q0[1],
         q1[0]*q0[1] - q1[1]*q0[0] + q1[2]*q0[3] + q1[3]*q0[2],
        -q1[0]*q0[0] - q1[1]*q0[1] - q1[2]*q0[2] + q1[3]*q0[3]), dtype=np.float64)


class PositionController(Addon):

    def __init__(self, parent, config):
        super(PositionController, self).__init__()

        self.uid = parent.uid
        self.gripper_id = [p.getJointInfo(self.uid, i)[1].decode('utf-8') for i in range(p.getNumJoints(self.uid))].index(config.get('end_effector_frame'))

        self.joint_ids = [i for i in range(p.getNumJoints(self.uid)) if p.getJointInfo(self.uid, i)[3] > -1]
        self.rest_position = config.get('rest_position')
        jointInfo = [p.getJointInfo(self.uid, i) for i in self.joint_ids]
        self.joint_position_lower_limit = [info[8] for info in jointInfo]
        self.joint_position_upper_limit = [info[9] for info in jointInfo]
        self.torque_limit = [info[10] for info in jointInfo]

        self.action_space = spaces.Dict({
            'position': spaces.Box(-1.0, 1.0, shape=(3,), dtype='float32'),
            'orientation': spaces.Box(-1.0, 1.0, shape=(3,), dtype='float32')
        })

        self.reset()

    def reset(self):
        for joint_id, angle in zip(self.joint_ids, self.rest_position):
            p.resetJointState(self.uid, joint_id, angle)

        self.target_state = [np.array(s) for s in p.getLinkState(self.uid, self.gripper_id)]

    def update(self, action):
        self.target_state[0] = self.target_state[0] + action['position']
        self.target_state[1] = quaternion_multiply(self.target_state[1], p.getQuaternionFromEuler(action['orientation']))

        joint_poses = p.calculateInverseKinematics(
            self.uid,
            self.gripper_id,
            self.target_state[0],
            self.target_state[1],
            self.joint_position_lower_limit,
            self.joint_position_upper_limit,
            np.subtract(self.joint_position_upper_limit, self.joint_position_lower_limit).tolist(),
            self.rest_position
        )

        p.setJointMotorControlArray(
            self.uid,
            self.joint_ids,
            p.POSITION_CONTROL,
            joint_poses,
            targetVelocities=[0.0] * len(joint_poses),
            forces=self.torque_limit,
            positionGains=[0.03] * len(joint_poses),
            velocityGains=[1.0] * len(joint_poses)
        )
