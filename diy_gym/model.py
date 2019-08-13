import pybullet as p
import pybullet_data
import os
from gym import spaces
from collections import OrderedDict
from .addons.addon import AddonFactory, Receptor

urdf_path = ['', os.path.join(os.path.dirname(os.path.abspath(__file__)), 'data/'), pybullet_data.getDataPath()]


class Model(Receptor):
    """Models represent any object to be spawned in the simulation environment.

    Models are described using a URDF so anything for which you can find one of those can be spawned and interacted with in the
    gym. Models, along with the environment itself serve as a receptor onto which addons can be attached. As such models themselves
    don't implement a whole lot of functionality, they mainly allow the physical characteristics of the model to be configured.

    Note that it's also possible to attach a model to another model. An example of a config file that does this is as follows:
    ```
    robot_arm:
        model: ur5/ur5_robot.urdf

        child_model:
            model: two_fingered_gripper.urdf
            parent_frame: ee_fixed_joint
            xyz: [0,0,0.01] # attach the gripper at a 1cm offset along the z-axis of the end effector frame
    ```

    Args:
        config_file (string): A file path pointing to the configuration file describing the environment
        parent (:obj:Model, optional): if this model is a child of another model then parent will be a reference to that model


    Configs:
        model (str): a path to a URDF file relative to any of the directories listed under `urdf_path` above.
        xyz (list of floats, optional, [0,0,0]): the position at which to spawn the model
        rpy (list of floats, optional, [0,0,0]): the orientation at which to spawn the model expressed in euler angles
        scale (float, optional, 1): resizes the model (i.e setting to 2 will double its size)
        use_fixed_base (bool, optional, False): if True, the model will be fixed in space and not subject to gravity
            or collisions
        mass (list of floats, optional): overrides the mass of the model defined in the URDF
        color (list of floats, optional): overrides the color of the model defined in the URDF
        parent_frame (str, optional): if this model is a child of another model then parent_frame defines
            the frame on the parent that this model will be attached to (default is the parent's base frame)
        child_frame (str, optional): if this model is a child of another model then child_frame defines the
            frame on this model that will be coupled to the parent frame (default is the base frame)
    """
    def __init__(self, config, parent=None):
        Receptor.__init__(self)

        self.name = config.name

        self.position = config.get('xyz', [0., 0., 0.])
        self.orientation = p.getQuaternionFromEuler(config.get('rpy', [0., 0., 0.]))
        use_fixed_base = config.get('use_fixed_base', False)
        scale = config.get('scale', 1.0)
        urdf = config.get('model')

        try:
            full_urdf_path = next(
                os.path.join(path, urdf) for path in urdf_path if os.path.isfile(os.path.join(path, urdf)))
        except StopIteration:
            raise ValueError('Could not find URDF: ' + urdf)

        self.uid = p.loadURDF(full_urdf_path, useFixedBase=use_fixed_base, globalScaling=scale)

        if parent is None:
            p.resetBasePositionAndOrientation(self.uid, self.position, self.orientation)
        else:
            parent_frame_id = parent.get_frame_id(config.get('parent_frame')) if 'parent_frame' in config else -1
            child_frame_id = self.get_frame_id(config.get('child_frame')) if 'child_frame' in config else -1

            pose = p.getLinkState(parent.uid, parent_frame_id)[:2] if parent_frame_id != -1 else p.getBasePositionAndOrientation(parent.uid)[:2]
            p.resetBasePositionAndOrientation(self.uid, *pose)

            p.createConstraint(parent.uid, parent_frame_id, self.uid, child_frame_id, p.JOINT_FIXED, [0, 0, 1],
                               self.position, [0, 0, 0], self.orientation, [0, 0, 0, 1])

        if 'mass' in config:
            p.changeDynamics(self.uid, -1, mass=config.get('mass'))

        if 'color' in config:
            p.changeVisualShape(self.uid, -1, rgbaColor=config.get('color'))

        self.addons = OrderedDict(
            sorted(
                {child.name: AddonFactory.build(child.get('addon'), self, child)
                 for child in config.find_all('addon')}.items(),
                key=lambda t: t[0]))
        self.models = OrderedDict(
            sorted({child.name: Model(child, self)
                    for child in config.find_all('model')}.items(), key=lambda t: t[0]))

    def get_frame_id(self, frame):
        frames = [p.getJointInfo(self.uid, i)[1].decode('utf-8') for i in range(p.getNumJoints(self.uid))]
        return frames.index(frame) if frame in frames else -1

    def get_transform(self, frame_id=-1):
        if frame_id >= 0:
            link_state = p.getLinkState(self.uid, frame_id)
            xyz, rot = link_state[4], link_state[5]
        else:
            model_state = p.getBasePositionAndOrientation(self.uid)
            xyz, rot = model_state[0], model_state[1]

        return xyz, rot
