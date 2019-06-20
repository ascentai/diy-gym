import numpy as np
import math as m
import os
import pybullet as p
from gym import spaces
from diy_gym import DIYGym
from diy_gym.addons.addon import Addon, AddonFactory


class Propellor(Addon):
    """Propellor defines an addon that can be used to apply an external force to the base a model.
        It's used here to show how to define a custom addon and integrate it into DIYGym.
    """
    def __init__(self, parent, config):
        super(Propellor, self).__init__(parent, config)

        # store the uid of the parent model for later
        self.uid = parent.uid

        # get the config for the position of the force or default to [0 0 0] if it's not specified
        self.frame_id = parent.get_frame_id(config.get('frame'))
        self.max_thrust = config.get('max_thrust', 20.0)
        self.max_torque = config.get('max_torque', 0.1) * (1.0 if config.get('rotor_direction') == 'CCW' else -1.0)
        self.rotor_speed = 0.0
        self.spool_up_rate = 0.1

        # define the space for the actions this addon expects to receive and the observations it will generate
        self.observation_space = spaces.Box(0.0, 1.0, shape=(1, ), dtype='float32')
        self.action_space = spaces.Box(0.0, 1.0, shape=(1, ), dtype='float32')

    def update(self, action):
        """Call the pybullet function to apply the desired force to this model.
        """
        self.rotor_speed = self.rotor_speed + (action - self.rotor_speed) * self.spool_up_rate
        p.applyExternalForce(self.uid, self.frame_id, [0, 0, self.max_thrust * self.rotor_speed], [0, 0, 0], p.LINK_FRAME)
        p.applyExternalTorque(self.uid, self.frame_id, [0, 0, self.max_torque * self.rotor_speed], p.LINK_FRAME)

    def observe(self):
        return self.rotor_speed


class FellOver(Addon):
    """FellOver defines an addon that can be used to apply an external force to the base a model.
    It's used here to show how to define a custom addon and integrate it into DIYGym.
    """
    def __init__(self, parent, config):
        super(FellOver, self).__init__(parent, config)

        # store the uid of the parent model for later
        self.uid = parent.uid

    def is_terminal(self):
        pose = p.getBasePositionAndOrientation(self.uid)
        return 2. * m.atan2(np.linalg.norm(pose[1][:3]), abs(pose[1][3])) > m.radians(10)


AddonFactory.register_addon('propellor', Propellor)
AddonFactory.register_addon('fell_over', FellOver)


def quaternion_multiply(q1, q0):
    return np.array([
        q1[0] * q0[3] + q1[1] * q0[2] - q1[2] * q0[1] + q1[3] * q0[0],
        -q1[0] * q0[2] + q1[1] * q0[3] + q1[2] * q0[0] + q1[3] * q0[1],
        q1[0] * q0[1] - q1[1] * q0[0] + q1[2] * q0[3] + q1[3] * q0[2],
        -q1[0] * q0[0] - q1[1] * q0[1] - q1[2] * q0[2] + q1[3] * q0[3]
    ])


if __name__ == '__main__':

    config_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'drone_pilot.yaml')
    env = DIYGym(config_file)

    terminal = True

    position_P_gain = 0.0005
    attitude_P_gain = 0.01
    altitude_P_gain = 0.75
    altitude_I_gain = 0.00075
    altitude_D_gain = 0.5

    # thrust =       motor1 + motor2 + motor3 + motor4
    # roll_torque =         - motor2          + motor4
    # pitch_torque = motor1          - motor3
    # yaw_torque =   motor1 - motor2 + motor3 - motor4
    thrust_torque_to_motor_speeds = np.linalg.inv(np.array([[1, 1, 1, 1], [0, -1, 0, 1], [1, 0, -1, 0], [1, -1, 1, -1]]))

    while True:

        if terminal:
            observation = env.reset()
            altitude_error_integral = 0.

        # get the position of the target wrt the drone
        position_error, _ = p.multiplyTransforms(
            *p.invertTransform(observation['drone']['pose']['position'], p.getQuaternionFromEuler(observation['drone']['pose']['orientation'])),
            observation['target']['pose']['position'],
            p.getQuaternionFromEuler(observation['target']['pose']['orientation'])
        )

        # Position control: calculate a bank angle so as to move the drone to the target
        roll = position_error[1] * position_P_gain
        pitch = -position_error[0] * position_P_gain
        q_ref = p.getQuaternionFromEuler([roll, pitch, 0])

        # Altitude control: calculate thrust necessary to achieve desired altitude
        altitude_error_integral += position_error[2]
        thrust = position_error[2] * altitude_P_gain + altitude_error_integral * altitude_I_gain - observation['drone']['pose']['velocity'][2] * altitude_D_gain

        # Attitude control: calculate torques necessary to achieve desired attitude
        torques = -quaternion_multiply(q_ref, p.getQuaternionFromEuler(observation['drone']['pose']['orientation']))[:3] * attitude_P_gain

        if observation['drone']['pose']['position'][2] < 2.0:
            torques[:] = 0

        # Motor speed control: calculate motor speeds required to achieve desired thrust/torques
        motor_speeds = thrust_torque_to_motor_speeds.dot([thrust, *torques])

        action = {
            'drone': {
                'motor1': motor_speeds[0],
                'motor2': motor_speeds[1],
                'motor3': motor_speeds[2],
                'motor4': motor_speeds[3]
            }
        }

        observation, reward, terminal, info = env.step(action)
