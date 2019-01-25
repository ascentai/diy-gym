import pybullet as p

from gym import spaces
from robo_gym.addons.controllers.controller_interface import ControllerInterface


class JointController(ControllerInterface):

    def __init__(self, parent, config):
        super(JointController, self).__init__(parent, config)

        self.gripper_id = [p.getJointInfo(self.uid, i)[1].decode('utf-8') for i in
                           range(p.getNumJoints(self.uid))].index(config.get('end_effector_frame'))

        self.joint_ids = sorted([joint_info[0] for joint_info in self.joint_info_dict.values()])
        self.torque_limit = [p.getJointInfo(self.uid, joint_id)[10] for joint_id in self.joint_ids]

        self.action_space = spaces.Dict({
            'position': spaces.Box(-1.0, 1.0, shape=(len(self.joint_ids),), dtype='float32'),
        })

    def reset(self):
        self.target_states = self.rest_position[:]

        for joint_id, angle in zip(self.joint_ids, self.rest_position):
            p.resetJointState(self.uid, joint_id, angle)

    def update(self, action):
        joint_poses = self.target_states + action['position']

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
