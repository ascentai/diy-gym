import numpy as np
import pybullet as p
from gym import spaces
from ..addon import Addon


class GripperController(Addon):
    def __init__(self, parent, config):
        super(GripperController, self).__init__()

        self.uid = parent.uid
        self.joint_dict = self.joint_name_to_id
        self.rest_position = config.get('rest_position')

        self.operating_width = 0.7
        self.target = self.rest_position
        self.torque_limit = p.getJointInfo(self.uid, self.joint_dict['left_outer_knuckle_joint'])[10]

        self.action_space = spaces.Dict(
            {'position': spaces.Box(0.0, self.operating_width, shape=(1,), dtype='float32')})

    def reset(self):
        self.target = self.rest_position
        p.resetJointState(self.uid, self.joint_dict['left_outer_knuckle_joint'], self.rest_position)
        p.resetJointState(self.uid, self.joint_dict['right_outer_knuckle_joint'], self.rest_position)

    def update(self, action):

        if self.target <= self.operating_width:
            self.target += action['position']

        # Update left claw
        p.setJointMotorControl2(self.uid, self.joint_dict['left_outer_knuckle_joint'], p.POSITION_CONTROL, self.target, force=self.torque_limit)
        p.setJointMotorControl2(self.uid, self.joint_dict['left_inner_knuckle_joint'], p.POSITION_CONTROL, self.target, force=self.torque_limit)
        p.setJointMotorControl2(self.uid, self.joint_dict['left_inner_finger_joint' ], p.POSITION_CONTROL, self.target, force=self.torque_limit)

        # Update right claw
        p.setJointMotorControl2(self.uid, self.joint_dict['right_outer_knuckle_joint'], p.POSITION_CONTROL, self.target, force=self.torque_limit)
        p.setJointMotorControl2(self.uid, self.joint_dict['right_inner_knuckle_joint'], p.POSITION_CONTROL, self.target, force=self.torque_limit)
        p.setJointMotorControl2(self.uid, self.joint_dict['right_inner_finger_joint' ], p.POSITION_CONTROL, self.target, force=self.torque_limit)

    @property
    def joint_name_to_id(self):
        name_to_id = {}

        for i in range(p.getNumJoints(self.uid)):
            joint_info = p.getJointInfo(self.uid, i)
            name_to_id[joint_info[1].decode('UTF-8')] = joint_info[0]

        return name_to_id
