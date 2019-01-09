import numpy as np
import pybullet as p
from gym import spaces
from plugins.plugin import Plugin


class JointController(Plugin):
    """Represents the Jaco robot"""

    def __init__(self, parent, config):
        super(JointController, self).__init__()

        self.uid = parent.uid
        self.gripper_id = [p.getJointInfo(self.uid, i)[1].decode('utf-8') for i in range(p.getNumJoints(self.uid))].index(config.get('end_effector_frame'))

        self.joint_ids = [i for i in range(p.getNumJoints(self.uid)) if p.getJointInfo(self.uid, i)[3] > -1]
        self.rest_position = config.get('rest_position')
        jointInfo = [p.getJointInfo(self.uid, i) for i in self.joint_ids]
        self.torque_limit = [info[10] for info in jointInfo]

        self.action_space = spaces.Dict({
            'position': spaces.Box(-1.0, 1.0, shape=(len(self.joint_ids),), dtype='float32'),
        })

    def reset(self):
        """Instantiate a Jaco in a default position, orientation and pose"""
        for joint_id, angle in zip(self.joint_ids, self.rest_position):
            p.resetJointState(self.uid, joint_id, angle)

        self.target_state = self.rest_position[:]

    def update(self, action):

        joint_poses = self.target_state + action['position']

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
