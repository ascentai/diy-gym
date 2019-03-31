import pybullet as p

from abc import abstractmethod

from robo_gym.addons.addon import Addon


class ControllerInterface(Addon):
    def __init__(self, parent, config):
        super(ControllerInterface, self).__init__(parent, config)

        self.uid = parent.uid

        # key = joint_id : value = joint_info
        self.joint_info_dict = {p.getJointInfo(self.uid, i)[1].decode('UTF-8'): p.getJointInfo(self.uid, i)
                                for i in range(p.getNumJoints(self.uid))}
        self.rest_position = config.get('rest_position')
        self.target_states = self.rest_position[:]

    @abstractmethod
    def reset(self):
        pass

    @abstractmethod
    def update(self, action):
        pass
