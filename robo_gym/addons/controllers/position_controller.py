import numpy as np
import pybullet as p
from gym import spaces
from robo_gym.addons.controllers.controller_interface import ControllerInterface


def quaternion_multiply(q1, q0):
    """Return multiplication of two quaternions."""
    return np.array((
         q1[0]*q0[3] + q1[1]*q0[2] - q1[2]*q0[1] + q1[3]*q0[0],
        -q1[0]*q0[2] + q1[1]*q0[3] + q1[2]*q0[0] + q1[3]*q0[1],
         q1[0]*q0[1] - q1[1]*q0[0] + q1[2]*q0[3] + q1[3]*q0[2],
        -q1[0]*q0[0] - q1[1]*q0[1] - q1[2]*q0[2] + q1[3]*q0[3]), dtype=np.float64)


class PositionController(ControllerInterface):

    def __init__(self, parent, config):
        super(PositionController, self).__init__(parent, config)

        self.gripper_id = [p.getJointInfo(self.uid, i)[1].decode('utf-8')
                           for i in range(p.getNumJoints(self.uid))].index(config.get('end_effector_frame'))

        self.joint_ids = sorted([joint_info[0] for joint_info in self.joint_info_dict.values()
                                 if joint_info[0] < self.gripper_id and joint_info[3] > -1])

        self.joint_position_lower_limit = [p.getJointInfo(self.uid, joint_id)[8]  for joint_id in self.joint_ids]
        self.joint_position_upper_limit = [p.getJointInfo(self.uid, joint_id)[9]  for joint_id in self.joint_ids]
        self.torque_limit =               [p.getJointInfo(self.uid, joint_id)[10] for joint_id in self.joint_ids]

        self.target_states = [np.array(s) for s in p.getLinkState(self.uid, self.gripper_id)]

        self.action_space = spaces.Dict({
            'position': spaces.Box(-0.001, 0.001, shape=(3,), dtype='float32'),
            'orientation': spaces.Box(-0.001, 0.001, shape=(3,), dtype='float32')
        })

        self.reset()

    def reset(self):
        for joint_id, angle in zip(self.joint_ids, self.rest_position):
            p.resetJointState(self.uid, joint_id, angle)
        self.target_states = [np.array(s) for s in p.getLinkState(self.uid, self.gripper_id)]

    def update(self, action):
        self.target_states[0] = action['position']
        self.target_states[1] = quaternion_multiply(self.target_states[1], p.getQuaternionFromEuler(action['orientation']))

        joint_poses = p.calculateInverseKinematics(
            self.uid,
            self.gripper_id,
            self.target_states[0],
            self.target_states[1],
            self.joint_position_lower_limit,
            self.joint_position_upper_limit,
            np.subtract(self.joint_position_upper_limit, self.joint_position_lower_limit).tolist(),
            self.rest_position
        )[:self.gripper_id - 1]

        p.setJointMotorControlArray(
            self.uid,
            self.joint_ids,
            p.POSITION_CONTROL,
            joint_poses,
            targetVelocities=[0.0] * len(joint_poses),
            forces=self.torque_limit,
            positionGains=[0.0015] * len(joint_poses),
            velocityGains=[1.0] * len(joint_poses)
        )
