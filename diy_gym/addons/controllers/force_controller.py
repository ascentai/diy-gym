import pybullet as p

from gym import spaces
from diy_gym.addons.addon import Addon
import numpy as np

class ForceController(Addon):
    """JointController
    """
    def __init__(self, parent, config):
        super(ForceController, self).__init__(parent, config)

        self.uid = parent.uid

        joint_info = [p.getJointInfo(self.uid, i) for i in range(p.getNumJoints(self.uid))]
        joints = [info[1].decode('UTF-8') for info in joint_info]

        self.joint_ids = [info[0] for info in joint_info if info[1].decode('UTF-8') in joints and info[3] > -1]
        self.rest_position = config.get('rest_position', [0] * len(self.joint_ids))

        self.torque_limit = [p.getJointInfo(self.uid, joint_id)[10] for joint_id in self.joint_ids]
        self.action_space = spaces.Box(-0.5, 0.5, shape=(len(self.joint_ids), ), dtype='float32')

    def wrench2torque(self, wrench):
        joint_info = [p.getJointInfo(self.uid, i) for i in range(p.getNumJoints(self.uid))]
        end_effector_joint_id = [info[1].decode('UTF-8') for info in joint_info].index('ee_fixed_joint')
        link_state = p.getLinkState(self.uid, end_effector_joint_id, computeForwardKinematics=True,
                                    computeLinkVelocity=False)
        local_position = link_state[2]
        joint_states = p.getJointStates(self.uid, self.joint_ids)
        joint_position = [state[0] for state in joint_states]
        joint_velocity = [state[1] for state in joint_states]
        zero_vec = [0, 0, 0, 0, 0, 0]
        linear_jacobian, angular_jacobian = np.array(
            p.calculateJacobian(
                bodyUniqueId=self.uid,
                linkIndex=end_effector_joint_id,
                localPosition=local_position,
                objPositions=joint_position,
                objVelocities=joint_velocity,
                objAccelerations=zero_vec)
        )
        jacobian = np.vstack((linear_jacobian, angular_jacobian))
        grav_component = p.calculateInverseDynamics(self.uid, zero_vec, zero_vec, zero_vec)
        return np.dot(jacobian.transpose(), wrench) + grav_component

    def reset(self):
        for joint_id, angle in zip(self.joint_ids, self.rest_position):
            p.resetJointState(self.uid, joint_id, angle)

    def update(self, action):
        kwargs = {}

        joint_torques = self.wrench2torque(action)

        print([1.0] * len(action))
        p.setJointMotorControlArray(self.uid,
                                    self.joint_ids,
                                    p.TORQUE_CONTROL,
                                    forces=joint_torques,
                                    positionGains=[0.03] * len(action),
                                    velocityGains=[1.0] * len(action))
