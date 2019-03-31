import numpy as np
import pybullet as p
from gym import spaces
from robo_gym.addons.controllers.controller_interface import ControllerInterface


class ForceController(ControllerInterface):

    def __init__(self, parent, config):
        super(ForceController, self).__init__(parent, config)

        self.gripper_id = [p.getJointInfo(self.uid, i)[1].decode('utf-8')
                           for i in range(p.getNumJoints(self.uid))].index(config.get('end_effector_frame'))
        self.joint_ids = sorted([joint_info[0] for joint_info in self.joint_info_dict.values() if joint_info[3] > -1])

        # Get parameters from config file
        self.force_range    = config.get('force_range', [-250.0, 250.0])
        self.disable_motors = config.get('disable_motors', False)
        self.k_p            = config.get('k_p', 1)

        if self.disable_motors:
            # Disable the motors for torque control
            p.setJointMotorControlArray(self.uid, self.joint_ids[:6], p.VELOCITY_CONTROL, forces=[0.0 for i in range(6)])

        # Enable force sensor on end-effector
        p.enableJointForceTorqueSensor(self.uid, self.gripper_id, enableSensor=1)

        self.action_space = spaces.Dict({
            'wrench': spaces.Box(self.force_range[0], self.force_range[1], shape=(6,), dtype='float32'),
        })

        self.reset()

    def reset(self):
        for joint_id, angle in zip(self.joint_ids, self.rest_position):
            p.resetJointState(self.uid, joint_id, angle)

    def update(self, action):
        p.setJointMotorControlArray(
            self.uid,
            self.joint_ids[:6],
            p.TORQUE_CONTROL,
            forces=self.calculate_torques(action)
        )

    def calculate_torques(self, action):
        # Read joint states
        joint_states = p.getJointStates(self.uid, self.joint_ids)

        # Get readings from the wrist's load sensor
        ee_forces = np.array(p.getJointState(self.uid, self.gripper_id)[2])

        # Set target forces (x, y, z, roll, pitch, yaw)
        target_forces = self.k_p * (action['wrench'] - ee_forces)

        # Get prev timestep positions and velocities
        q_pos = [state[0] for state in joint_states]
        q_vel = [state[1] for state in joint_states]

        # Solve for Jacobian
        zero_vector = [0.0 for i in range(len(q_pos))]
        link_local_pos = p.getLinkState(self.uid, self.gripper_id)[2]

        J = np.vstack(p.calculateJacobian(self.uid, self.gripper_id, link_local_pos, q_pos, zero_vector, zero_vector)).tolist()

        # Solve for acceleration
        q_acc = np.linalg.lstsq(J, target_forces, rcond=None)[0].tolist()

        # Use inverse dynamics to get torques
        torques = p.calculateInverseDynamics(self.uid, q_pos, q_vel, q_acc)

        # Only passing torques for joints 1-6, joints 7+ corresponds to the gripper
        return torques[:6]
