import numpy as np

from ..addon import Addon
from ..sensors.object_state_sensor import ObjectStateSensor
from ..sensors.end_effector_state_sensor import EndEffectorStateSensor


class GripperEnvReward(Addon):
    def __init__(self, parent, config):
        super(GripperEnvReward, self).__init__(parent, config)

        self.target_obj = parent.models[config.get('target_object')]
        self.ur_robot = parent.models['ur5']

        # according to table's urdf
        self.table_height = 0.62

        self.prev_gripper_dist = self.calculate_distance(ObjectStateSensor(self.target_obj, config).observe()['position'],
                                                         EndEffectorStateSensor(self.ur_robot, config).observe()['position'])
        self.prev_obj_pos = ObjectStateSensor(self.target_obj, config).observe()['position']

        self.config = config
        self.done = False

    def reward(self):
        # get updated observations
        object_state_obs = ObjectStateSensor(self.target_obj, self.config).observe()
        ee_state_obs     = EndEffectorStateSensor(self.ur_robot, self.config).observe()

        curr_obj_pos     = object_state_obs['position']

        # calculate target position
        target_pos = [float(i) for i in curr_obj_pos]
        target_pos[2] += 0.25

        # reference [https://github.com/ascentai/yamaha_demo/blob/new-demo-testing/YamahaSimulator.py]
        # compute the gripper movement reward
        curr_gripper_dist = self.calculate_distance(target_pos, ee_state_obs['position'])
        # gripper_reward = 1000 * (self.prev_gripper_dist - curr_gripper_dist)

        # compute object movement reward
        object_reward = 1000 * self.calculate_distance(self.prev_obj_pos, curr_obj_pos) \
            if curr_gripper_dist < 0.3 else 0

        # compute task reward see terminal state is reached
        task_reward = 0

        if object_state_obs['position'][2] > (self.table_height + 0.2) and curr_gripper_dist < 0.3:
            task_reward = 100000
            self.done = True

        # update previous positions
        self.prev_gripper_dist = curr_gripper_dist

        # account for the table height
        self.prev_obj_pos      = curr_obj_pos

        return object_reward + task_reward

    def is_terminal(self):
        return self.done

    @staticmethod
    def calculate_distance(a, b):
        return np.sqrt(np.sum(np.power(np.array(a)-np.array(b), 2)))

