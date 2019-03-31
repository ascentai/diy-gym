import pybullet as p

from gym import spaces

from ..addon import Addon


class ObjectStateSensor(Addon):

    def __init__(self, parent, config, link_name=None, force_enabled=False):
        super(ObjectStateSensor, self).__init__(parent, config)

        self.uid = parent.uid
        self.link_name = config.get('link_name') if config.has_key('link_name') else link_name

        self.link_id = -1 if self.link_name is None else [p.getJointInfo(self.uid, i)[1].decode('utf-8')
                                                          for i in range(p.getNumJoints(self.uid))].index(self.link_name)

        self.enable_force = config.get('enable_force', force_enabled)

        if self.enable_force:
            p.enableJointForceTorqueSensor(self.uid, self.link_id, enableSensor=self.enable_force)

        self.observation_space = spaces.Dict({
            'position'   : spaces.Box(-10, 10, shape=(3,), dtype='float32'),
            'orientation': spaces.Box(-10, 10, shape=(3,), dtype='float32'),
        })

    def observe(self):
        observation = list(p.getLinkState(self.uid, self.link_id) if self.link_id is not -1 else p.getBasePositionAndOrientation(self.uid))
        obs_dict = {'position'   : observation[0],
                    'orientation': p.getEulerFromQuaternion(observation[1])}

        if self.enable_force:
            obs_dict['forces'] = p.getJointState(self.uid, self.link_id)[2]

        return obs_dict
