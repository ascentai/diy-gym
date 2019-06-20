DIYGym
========

DIYGym is a framework for creating reinforcement learning environments using pybullet. It's designed to simplify the process of parameterising an environment, defining its observations, actions and reward signals and bottling all of that functionality up into an OpenAI style gym interface. It's especially useful replicating physical robot setups in simulation and randomising the parameters of that simulation for sim-to-real transfer.

## How it Works:

DIYGym wraps pybullet up in a slightly higher level framework loosely inspired by Gazebo's SDFs and Plugins. In particular, it defines:

* A config file which declares and parameterises all the objects to be spawned in the simulated environment
* A set of add-ons which can be attached to an object to control, observe, derive reward from or just generally interact with that object during a simulation.

Once configured, DIYGym presents itself as any other OpenAI-style gym environment. Create an environment like this:
```python
env = DIYGym(path_to_config_file)
```
Check out it's action and observations spaces like so:
```python
print(env.observation_space)
random_action = env.action_space.sample()
```
and run it in the usual loop like so:
```python
agent = YourLearningAgent()

while True:
    action = agent.act()

    observation, reward, is_terminal, info = env.step(action)

    agent.learn(observation, reward, is_terminal)

    if is_terminal:
        env.reset()
```
### Config Files:

To set up your environment you'll need to write own config file (or just adapt one of the [examples](https://github.com/ascentai/robo-gym/tree/master/examples)). Config files are yaml files that contain all the information DIYGym requires to describe an environment including what objects it should contain, how they should behave and how a learning agent can interact with them.

The file itself contains any number of top level items called `model`s. A model declares an object that will be spawned in the simulation environment. DIYGym identifies models in the config file as yaml dictionaries that contain an item with the key `model`; the value of which should be a file path to a URDF describing the geometry of the object to be spawned. Note that DIYGym will search for URDFs in the data folder of this package, the pybullet data folder or you can also just specify the absolute path directly in the config file. There are a few additional params that can be supplied to a model to describe its pose, scale etc (see `model.py` for more).

To see what this looks like, let's create an environment in which a Jaco robot arm sits on a table in front of a tiny R2D2 robot using the following config file:

```yaml
plane:
    model: plane.urdf

table:
    model: table/table.urdf
    xyz: [0.0, 0.4, 0.0]

r2d2:
    model: r2d2.urdf
    xyz: [0.1, 0.5, 0.7]
    rpy: [0.0, 0.0, 3.1415]
    scale: 0.1

robot:
    model: jaco/j2s7s300_standalone.urdf
    xyz: [0.0, 0.0, 0.65]
```
If we then instantiate a DIYGym passing in the config file above we'll be met with something that looks like the following:
```python
useless_env = DIYGym(path_to_that_config_file)
```
![limp_jaco](https://user-images.githubusercontent.com/38680667/51458000-f1c48980-1d96-11e9-8724-dc44ad730b00.png)

As promised, this environment does match the one described above, but in it's current state it's pretty useless; there's no way to control any of the models or take any observations from them, rewards are perpetually zero and episodes never terminate. If we run a few steps on the environment then this will become obvious:

```python
action = useless_env.action_space.sample()

print('Sampled action was:')
print(action)

observation, reward, terminal, info = useless_env.step(action)

print('Step returns:')
print(observation, reward, terminal, info)
```
prints:
```
Sampled action was:
{}
Step returns:
{} {} {} {}
```
To actually interact with an environment we'll need to define some add-ons.

### Add-ons:

An add-on is a chunk of code used to interact with the simulation whenever `step` or `reset` is called on a DIYGym instance; each add-on has the opportunity to retrieve information from the `action` dictionary passed to DIYGym's `step` method or to insert its information into the `observation`, `reward` or `is_terminal` dictionaries returned from `step`/`reset`.

Similar to models, add-ons are identified in the config file by DIYGym as being dictionaries containing an item with the key `addon`; the value of which should be a string referencing one of the addons that has been registered with DIYGym. Add-ons can be attached to either a model or the environment itself by adding their configs to the scope of the the desired parent in the config file.

DIYGym has a bunch of add-ons built-in to define common sensors, actuators and reward signals used in RL; if you find the built-in set of add-ons are lacking, you can pretty easily add your own too (more on that below). 

To see how this all works, let's add a few add-ons to our Jaco environment to make it a little more functional. Specifically, we'll modify the environment such that an agent can learn to pick up the R2D2 with the Jaco arm while minimising the joint torque expended in doing so. The updated config file will look like this:
```yaml
max_episode_steps: 50

plane:
    model: plane.urdf

table:
    model: table/table.urdf
    xyz: [0.0, 0.4, 0.0]

r2d2:
    model: r2d2.urdf
    xyz: [0.1, 0.5, 0.7]
    rpy: [0.0, 0.0, 3.1415]
    scale: 0.1

    arm_camera:
        addon: camera
        frame: left_tip_joint
        resolution: [200, 200]

robot:
    model: jaco/j2s7s300_standalone.urdf
    xyz: [0.0, 0.0, 0.65]

    controller:
        addon: position_controller
        rest_position: [0.0, 2.9, 0.0, 1.3, 4.2, 1.4, 0.0, 1.0, 1.0, 1.0]
        end_effector_frame: j2s7s300_joint_end_effector

    grab_r2d2:
        addon: reach_target
        target_position: [0.1, 0.5, 0.7]
        frame: j2s7s300_joint_end_effector

    lazy_robot:
        addon: electricity_cost
```

These additions will do the following:
* `controller` attached to the jaco arm will allow an agent to control the 3D pose of its end effector. 
* `arm_camera` will capture RGB and depth from the protruding arm of R2D2 and add those to the `observations` dictionary
* `grab_r2d2` will entice agents to pick up the R2D2 by calculating a penalty for the distance between it and the Jaco's end effector.
* `lazy_robot` will calculate a penalty for the amount of joint torque currently being applied by the jaco and this penalty will be included in the reward value returned by `step`

All this information is automatically reflected in DIYGym's action/observation spaces, as well as in the dictionaries it returns `step` and `reset`.

If we now instantiate a new DIYGym passing in this updated config file above we'll be met with something that looks like the following:

![jaco_smash](https://user-images.githubusercontent.com/38680667/51458026-0dc82b00-1d97-11e9-9c1f-6d63f73ccaf2.gif)

The objects returned by `step` will no longer be empty, but will be nested dictionaries keyed according to first the model name then the name of the addon that generated the corresponding piece of data. In the case of the updated Jaco environment, these dictionaries will have the following structure:
```
action
|---robot
    |---controller
        |---orientation <class 'numpy.ndarray'>
        |---position <class 'numpy.ndarray'>
```
```
observation
|---robot
|   |---grab_r2d2
|       |---achieved_position <class 'numpy.ndarray'>
|       |---target_position <class 'numpy.ndarray'>
|---r2d2
    |---arm_camera
        |---depth <class 'numpy.ndarray'>
        |---rgb <class 'numpy.ndarray'>
```
`reward` and `terminal` are also represented by dictionaries which is a slight departure from the usual gym interface. If you'd rather all their fields be combined you can set the config `sum_rewards`, `terminal_if_any` or `terminal_if_all` to true in the environment node of the config file (see [examples/ur_high_5](https://github.com/ascentai/robo-gym/tree/master/examples/ur_high_5) for an example).

```
reward
|---robot
    |---lazy_robot <class 'float'>
    |---grab_r2d2 <class 'float'>
```
```
terminal
|---environment
|    |---episode_timer <class 'bool'>
|
|---robot
    |---grab_r2d2 <class 'bool'>
```

### Writing Your Own Addons:

If you need to customise your environment beyond what's possible with the built-in addons it's pretty easy to add your own. To do so, just subclass `Addon` in addons/addon.py and follow the instructions in the docstrings to fill out your desired callbacks.

Once you're satisfied with your addon you can register it with DIYGym using the `AddonFactory` class like so:
```python
from diy_gym.addons.addon import Addon, AddonFactory

class MyAddon(Addon):
    ...

AddonFactory.register_addon('my_addon', MyAddon)
```
This will add your addon to a dictionary maintained by the factory so choose a name that doesn't clash with any of built-in addons or any others that you've defined.

Once the addon is added you can refer to it in a config file just as you would any other addon and use that file to create a DIYGym:
```python
env = DIYGym(path_to_config_file)
```
For an actual example of how to add a addon to DIYGym, check out the [jaco_on_a_table](https://github.com/ascentai/robo-gym/tree/master/examples/jaco_on_a_table) example.

## Installation:

It's best to work out of a conda environment. If you haven't already, download and install [conda](https://www.anaconda.com/download/). Once you're done, make yourself an environment using python3.5 and activate it like so:
```
conda create --name robo-gym python=3.5
source activate robo-gym
```
Now navigate to wherever you cloned DIYGym and install its requirements followed by the DIYGym itself:
```
cd $PATH_TO_ROBOGYM
pip install -r requirements.txt
pip install -e .
```
To test your installation, you can run any of the environments in the examples folder:
```
cd $PATH_TO_ROBOGYM/examples/ur_high_5
python ur_high_5.py
```
