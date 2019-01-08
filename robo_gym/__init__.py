from .jaco import Jaco
from .jaco_gym import JacoGym
from .jaco_ascent_gym import AscentJacoGym

from gym.envs.registration import register

ctrl = {'cartesian': 'Cart',
        'position': 'Pos',
        'velocity': 'Vel',
        'torque': 'Torque'}

discrete = [(False, ''), (True, 'Discrete')]

for control_modality in ctrl.keys():
    for reward_type in ['dense', 'sparse']:
        for is_discrete, discrete_name in discrete:
            if is_discrete and control_modality in ['velocity', 'torque']:
                continue
            name = 'JacoGymReach{}{}{}-v0'.format(discrete_name,
                                                  ctrl[control_modality],
                                                  reward_type.capitalize())
            register(id=name, entry_point='jaco_gym:JacoGym',
                    kwargs={'control_type': control_modality,
                            'reward_type': reward_type,
                            'discrete_control': is_discrete})
