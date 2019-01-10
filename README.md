RoboGym
========

RoboGym is a flexible simulation environment built on top of pybullet to drive research on robot arms here at Ascent. It's intended to be a reinforcement learning environment but it's probably flexible enough to be useful for research outside of RL too though, so don't be deterred if RL isn't your thing.

RoboGym is designed to easily support all the weird and wonderful research ideas you (yes, you!) might have and to save a lot of the legwork involved getting a robot arm up and running in pybullet. With very little effort, RoboGym can be set up to simulate a huge variety of environments, all complete with OpenAI gym interfaces and fully defined action and observation spaces:

From our trusty Jaco arm:

![getting_jaco_ed_up](https://user-images.githubusercontent.com/38680667/50953806-d8e6e900-14f7-11e9-9f93-ca190f9c6b65.gif)

To a fairly faithful recreation of the 8f robot room - including some molexes, the rs007n and its gripper mounted RGBD camera:

![robot_room](https://user-images.githubusercontent.com/38680667/50953808-d8e6e900-14f7-11e9-9ee0-3cadf0f7fcb6.gif)

Even to environments with multiple robot arms, mutliple reward signals and multiple episode termination conditions:

![ur_high_5](https://user-images.githubusercontent.com/38680667/50953809-d97f7f80-14f7-11e9-8195-727cd21e8e5b.gif)

## How it Works:

RoboGym's design is loosely inspired by Gazebo:
* The environment can be fully described by an XML file which defines a bunch of models along with their positions and physical properties (a little bit like an SDF but without all the gazebo-specific mumbojumbo)
* Functionality can be easily extended by writing plugins to control the simulation directly through the pybullet API

The jaco arm and it's environment above are described by the following config file:

```
<?xml version="1.0"?>
<environment>
    <model name="plane">
        <urdf>plane.urdf</urdf>
    </model>

    <model name="table">
        <xyz>0.0 0.4 0.0</xyz>
        <urdf>table/table.urdf</urdf>
    </model>

    <model name="purple_thing">
        <xyz>0.0 0.5 0.7</xyz>
        <urdf>random_urdfs/865/865.urdf</urdf>
    </model>

    <model name="robot">
        <urdf>jaco/j2s7s300_standalone.urdf</urdf>
        <xyz>0.0 0.0 0.65</xyz>

        <plugin name="controller" type="position_controller">
            <rest_position>0.0 2.9 0.0 1.3 4.2 1.4 0.0 1.0 1.0 1.0</rest_position>
            <end_effector_frame>j2s7s300_joint_end_effector</end_effector_frame>
        </plugin>

        <plugin name="joint_state" type="joint_state_sensor"/>
    </model>
</environment>
```
