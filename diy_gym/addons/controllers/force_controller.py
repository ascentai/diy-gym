import pybullet as p

from gym import spaces
from diy_gym.addons.addon import Addon

class AdmittanceController(Addon):
    def __init__(self, parent, config):
        super(ForceController, self).__init__(parent, config)

        self.uid = parent.uid

        self.end_frame = parent.get_frame_id(config.get('end_effector'))
        self.kp = config.get('p_gain', 0.001)
        self.kd = config.get('d_gain', 0.01)

        joint_info = [p.getJointInfo(self.uid, i) for i in range(p.getNumJoints(self.uid))]
        self.joint_ids = [info[0] for info in joint_info if info[0] <= self.end_frame and info[3] > -1]

        self.rest_position = config.get('rest_position', [0] * len(self.joint_ids))

        self.action_space = spaces.Dict({
            'force': spaces.Box(-5, 5, shape=(3,), dtype='float32'),
            'torque': spaces.Box(-1., 1., shape=(3,), dtype='float32')})

        self.reset()

        p.setJointMotorControlArray(self.uid, self.joint_ids, p.VELOCITY_CONTROL, forces=[0]*len(self.joint_ids))

    def reset(self):
        for joint_id, angle in zip(self.joint_ids, self.rest_position):
            p.resetJointState(self.uid, joint_id, angle)

    def update(self, action):
        joint_positions = [state[0] for state in p.getJointStates(self.uid, self.joint_ids)]
        joint_velocities = [state[1] for state in p.getJointStates(self.uid, self.joint_ids)]
        n_joints = len(joint_positions)

        J = p.calculateJacobian(
            self.uid,
            self.end_frame,
            [0.,0.,0.],
            joint_positions,
            [0.]*n_joints,
            [0.]*n_joints)

        T_g = p.calculateInverseDynamics(self.uid, joint_positions, [0.]*n_joints, [0.]*n_joints)
        T_cmd = action['force'].dot(np.array(J[0])) + action['torque'].dot(np.array(J[1]))

        T_pos = (self.rest_position - np.array(joint_positions)) * self.kp
        T_vel = (np.array(joint_velocities)) * -self.kd

        p.setJointMotorControlArray(self.uid, self.joint_ids, p.TORQUE_CONTROL, forces=T_cmd+T_g+T_pos+T_vel)