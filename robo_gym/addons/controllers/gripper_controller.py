import copy
import pybullet as p
from gym import spaces
from robo_gym.addons.controllers.controller_interface import ControllerInterface


class Gripper2fController(ControllerInterface):
    def __init__(self, parent, config):
        super(Gripper2fController, self).__init__(parent, config)

        # Get all joints associated only with the gripper
        self.joint_ids = sorted([joint_id[0] for joint_id in self.joint_info_dict.values()
                                 if joint_id[0] > self.joint_info_dict[config.get('end_effector_frame')][0]
                                 and joint_id[3] > -1])
        self.action_space = spaces.Dict(
            {'position': spaces.Box(0.0, 0.75, shape=(1,), dtype='float32')})

    def reset(self):
        for joint_id, angle in zip(self.joint_ids, self.rest_position):
            p.resetJointState(self.uid, joint_id, angle)
        self.target_states = copy.deepcopy(self.rest_position)

    def update(self, action):
        # Controls the closing and opening of the gripper
        for i in range(len(self.target_states)):
            self.target_states[i] = action['position']

        p.setJointMotorControlArray(
            self.uid,
            self.joint_ids,
            p.POSITION_CONTROL,
            self.target_states,
            forces=[p.getJointInfo(self.uid, i)[10] for i in self.joint_ids],
        )

    # def observe(self):
    #     return {'position': self.target_states[0], 'is_open': self.is_open()}
    #
    # def is_open(self):
    #     return self.target_states[0] < 0.75


class Gripper3fController(ControllerInterface):
    def __init__(self, parent, config):
        super(Gripper3fController, self).__init__(parent, config)

        self.finger_joint_1_ids = sorted([joint_name[0] for joint_name in self.joint_info_dict.values()
                                          if '_joint_1' in joint_name[1].decode('UTF-8') and joint_name[3] > -1])
        self.finger_joint_2_ids = sorted([joint_name[0] for joint_name in self.joint_info_dict.values()
                                          if '_joint_2' in joint_name[1].decode('UTF-8') and joint_name[3] > -1])
        self.finger_joint_3_ids = sorted([joint_name[0] for joint_name in self.joint_info_dict.values()
                                          if '_joint_3' in joint_name[1].decode('UTF-8') and joint_name[3] > -1])
        self.palm_joint_ids = sorted([joint_name[0] for joint_name in self.joint_info_dict.values()
                                      if 'palm_finger_' in joint_name[1].decode('UTF-8') and joint_name[3] > -1])
        self.action_space = spaces.Dict(
            {'finger_joint1': spaces.Box(0.0495,  1.2218, shape=(3,), dtype='float32'),
             'finger_joint2': spaces.Box(0.00,    1.5708, shape=(3,), dtype='float32'),
             'finger_joint3': spaces.Box(-1.2218, 0.0495, shape=(3,), dtype='float32'),
             'palm_joint':    spaces.Box(-0.192,  0.1784, shape=(2,), dtype='float32'),
             }
        )

    def reset(self):

        # Reset finger joints 1
        for joint_id, angle in zip(self.finger_joint_1_ids, self.rest_position[0]):
            p.resetJointState(self.uid, joint_id, angle)

        # Reset finger joints 2
        for joint_id, angle in zip(self.finger_joint_2_ids, self.rest_position[1]):
            p.resetJointState(self.uid, joint_id, angle)

        # Reset finger joints 3
        for joint_id, angle in zip(self.finger_joint_3_ids, self.rest_position[2]):
            p.resetJointState(self.uid, joint_id, angle)

        # Reset palm joints
        for joint_id, angle in zip(self.palm_joint_ids, self.rest_position[3]):
            p.resetJointState(self.uid, joint_id, angle)

        self.target_states = copy.deepcopy(self.rest_position)

    def update(self, action):
        # Update finger joints 1
        for i in range(len(self.target_states[0])):
            self.target_states[0][i] = action['finger_joint1'][i]

        p.setJointMotorControlArray(
            self.uid,
            self.finger_joint_1_ids,
            p.POSITION_CONTROL,
            self.target_states[0],
            forces=[p.getJointInfo(self.uid, i)[10] for i in self.finger_joint_1_ids]
        )

        # Update finger joints 2
        for i in range(len(self.target_states[1])):
            self.target_states[1][i] = action['finger_joint2'][i]

        p.setJointMotorControlArray(
            self.uid,
            self.finger_joint_2_ids,
            p.POSITION_CONTROL,
            self.target_states[1],
            forces=[p.getJointInfo(self.uid, i)[10] for i in self.finger_joint_2_ids]
        )

        # Update finger joints 3
        for i in range(len(self.target_states[2])):
            self.target_states[2][i] = action['finger_joint3'][i]

        p.setJointMotorControlArray(
            self.uid,
            self.finger_joint_3_ids,
            p.POSITION_CONTROL,
            self.target_states[2],
            forces=[p.getJointInfo(self.uid, i)[10] for i in self.finger_joint_3_ids]
        )

        # Update palm joints
        for i in range(len(self.target_states[3])):
            self.target_states[3][i] = action['palm_joint'][i]

        p.setJointMotorControlArray(
            self.uid,
            self.palm_joint_ids,
            p.POSITION_CONTROL,
            self.target_states[3],
            forces=[p.getJointInfo(self.uid, i)[10] for i in self.palm_joint_ids]
        )
