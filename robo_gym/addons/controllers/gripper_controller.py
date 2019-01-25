import pybullet as p
from gym import spaces
from robo_gym.addons.controllers.controller_interface import ControllerInterface


class GripperController(ControllerInterface):
    def __init__(self, parent, config):
        super(GripperController, self).__init__(parent, config)

        self.gripper_operating_width = 0.7

        # Get all joints associated only with the gripper
        self.joint_ids = sorted([joint_id[0] for joint_id in self.joint_info_dict.values()
                                 if joint_id[0] > self.joint_info_dict[config.get('end_effector_frame')][0]])

        self.torque_limits = [p.getJointInfo(self.uid, i)[10] for i in self.joint_ids]
        self.action_space = spaces.Dict(
            {'position': spaces.Box(0.0, self.gripper_operating_width, shape=(1,), dtype='float32')})

    def reset(self):
        self.target_states = self.rest_position[:]
        for joint_id, angle in zip(self.joint_ids, self.rest_position):
            p.resetJointState(self.uid, joint_id, angle)

    def update(self, action):
        for i in range(len(self.target_states)):
            self.target_states[i] += action['position'] if self.target_states[i] < self.gripper_operating_width else 0

        p.setJointMotorControlArray(
            self.uid,
            self.joint_ids,
            p.POSITION_CONTROL,
            self.target_states,
            forces=self.torque_limits
        )

