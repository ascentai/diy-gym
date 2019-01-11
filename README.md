RoboGym
========

RoboGym is a flexible simulation environment built on top of pybullet to drive research on robot arms here at Ascent. It's intended to be a reinforcement learning environment but it's probably flexible enough to be useful for research outside of RL too though, so don't be deterred if RL isn't your thing.

RoboGym is designed to support all the weird and wonderful research ideas you (yes, you!) might have and to save a lot of the legwork involved getting a robot arm up and running in pybullet. With very little effort, RoboGym can be set up to simulate a huge variety of environments, all complete with OpenAI gym interfaces and fully defined action and observation spaces:

From our trusty Jaco arm:

![getting_jaco_ed_up](https://user-images.githubusercontent.com/38680667/50953806-d8e6e900-14f7-11e9-9f93-ca190f9c6b65.gif)

To a fairly faithful recreation of the 8f robot room - including some molexes, the rs007n and its gripper mounted RGBD camera:

![robot_room](https://user-images.githubusercontent.com/38680667/50953808-d8e6e900-14f7-11e9-9ee0-3cadf0f7fcb6.gif)

You can even define environments with multiple robot arms, reward signals and episode termination conditions:

![ur_high_5](https://user-images.githubusercontent.com/38680667/50953809-d97f7f80-14f7-11e9-8195-727cd21e8e5b.gif)

## Installation:


## How it Works:

Assuming you've got set up the environment the way you like it, then you can treat RoboGym like any other RL gym. Create an environment like this:
```
env = RoboGym(path_to_config_file)
```
Check out it's action and observations spaces like so:
```
print(env.observation_space)
random_action = env.action_space.sample()
```
and run it in the usual loop like so:
```
agent = YourLearningAgent()

while True:
    action = agent.act()

    observation, reward, is_terminal, info = env.step(action)

    agent.learn(observation, reward, is_terminal)

    if is_terminal:
        env.reset()
```
### Config Files:

To create your own environment you can write your own config file which can then be used to parameterise RoboGym.

Config files are XML files that contain one root node called `environment` which in turns contains any number of `model`s.

A model declares an object (be it a table or a robot arm) that should be spawned in the environment. Its only required field is a path to a URDF file that describes its geometry but there are a few additional params for describing pose, etc (see `model.py` for more).

RoboGym will search for URDFs in the data folder of this pacakge, the pybullet data folder or you can also just specify the absolute path.

As an example of what this looks like the jaco arm and it's environment above are described by the following config file:

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
    </model>
</environment>
```

### Plugins:

As it stands, the environment described above is pretty pointless since there's no way to control any of the models or take any observations from them. In fact if you print the action/observation spaces for the environment at this point they'll just be empty dictionaries.

To actually do things with an environment you can use what I'm calling a plugin. A plugin is a python object that can be attached to a model in order to interact with it in some way when the user calls `step()` or `reset()` on the RoboGym environment. There's a slew of built-in plugins that can be used to control robot arm end effectors, attach cameras to models and so forth so there's already a pretty decent set of environments that can be defined out-of-the-box.

To see how this works, let's add a couple of plugins to the jaco environment:

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

        <plugin name="purple_thing_camera" type="camera">
            <base_frame>baseLink</base_frame>
            <resolution>200 200</resolution>
        </plugin>

    </model>

    <model name="robot">
        <urdf>jaco/j2s7s300_standalone.urdf</urdf>
        <xyz>0.0 0.0 0.65</xyz>

        <plugin name="controller" type="position_controller">
            <rest_position>0.0 2.9 0.0 1.3 4.2 1.4 0.0 1.0 1.0 1.0</rest_position>
            <end_effector_frame>j2s7s300_joint_end_effector</end_effector_frame>
        </plugin>

        <plugin name="be_lazy" type="electricity_cost"/>

    </model>
</environment>
```
These additions will do the following:
* `purple_thing_camera` will take RGB and a depth image from the base_joint frame of the purple thing and add those observations to the dict returned when RoboGym is `step`'ed or `reset`
* `controller` will move the end effector of the jaco arm in cartesian mode in response actions passed to `step`
* `be_lazy` will calculate a penalty for the amount of joint torque currently being applied by the jaco and this penalty will be included in the reward value returned by `step`

### Writing Your Own Plugins:

If you need to customise your environment beyond what's possible with the built-in plugins it's pretty easy to add your own. To do so, just subclass `Plugin` in plugins/plugin.py and follow the instructions in the docstrings to fill out your desired callbacks.

Once you're satisfied with your plugin you can register it with RoboGym using the `PluginFactory` class like so:
```
class MyPlugin:
    ...

from robo_gym.plugins.plugin import PluginFactory

PluginFactory.add_plugin('my_plugin', MyPlugin)
```
This will add your plugin to a dictionary maintained by the factory so choose a name that doesn't clash with any of built-in plugins or any others that you've defined.

Once the plugin is added you can refer to it in a config file just as you would any other plugin and use that file to create a RoboGym:
```
env = RoboGym(path_to_config_file)
```
