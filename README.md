RoboGym
========

RoboGym is a framework for creating reinforcement learning environments using pybullet. It's designed to simplify the process of parameterising an environment, defining its observations, actions and reward signals and bottling all of that functionality up into an OpenAI style gym interface. It's especially useful replicating physical robot setups in simulation and randomising the parameters of that simulation for sim-to-real transfer.

RoboGym works by wrapping pybullet up in a slightly higher level framework loosely inspired by Gazebo's SDFs and Addons. In particular, it defines:

* A config file which declares and parameterises all the objects to spawn in the simulated environment
* A set of add-ons which can be attached to an object to control, observe, derive reward from or just generally interact with that object during a simulation.

RoboGym has a bunch of add-ons built-in to define common sensors, actuators and reward signals used in RL; if you find those are lacking, you can also pretty easily add your own (more on that below). As for the simulated objects themselves, basically whatever you can dig up a URDF for, you can incorporate straight into your environment. Point being, without too much effort you can define a really wide variety of RL environments with RoboGym; all complete with OpenAI gym interfaces and fully defined action and observation spaces.

![getting_jaco_ed_up](https://user-images.githubusercontent.com/38680667/51370511-b9287400-1b3a-11e9-957b-0e0206e79698.gif) ![ur_high_5](https://user-images.githubusercontent.com/38680667/50953809-d97f7f80-14f7-11e9-8195-727cd21e8e5b.gif)

## Installation:

It's best to work out of a conda environment. If you haven't already, download and install [conda](https://www.anaconda.com/download/). Once you're done, make yourself an environment using python3.5 and activate it like so:
```
conda create --name robo-gym python=3.5
source activate robo-gym
```
Now navigate to wherever you cloned RoboGym and install its requirements followed by the RoboGym itself:
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
## How it Works:

RoboGym works just like any other reinforcement learning gym. Create an environment like this:
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

To set up your environment you'll need to write own config file (or just adapt one of the [examples](https://github.com/ascentai/robo-gym/tree/master/examples)). Config files are .xml files that contain all the information RoboGym requires to set up an environment including what objects it should contain, how they should behave and how a learning agent can interact with them.

The file itself contains one root node called `environment` which in turns contains any number of `model`s. A model declares an object that will be spawned in the simulation environment. Its only required field is a path to a URDF file that describes its geometry but there are a few additional params for describing pose, etc (see `model.py` for more).

RoboGym will search for URDFs in the data folder of this pacakge, the pybullet data folder or you can also just specify the absolute path.

As an example of what this looks like we can define an environment in which a jaco arm sits on a table in front of a tiny R2D2 robot like so:

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

    <model name="r2d2">
        <xyz>0.0 0.5 0.7</xyz>
        <scale>0.1</scale>
        <urdf>r2d2.urdf</urdf>
    </model>

    <model name="robot">
        <urdf>jaco/j2s7s300_standalone.urdf</urdf>
        <xyz>0.0 0.0 0.65</xyz>
    </model>
</environment>
```

### Addons:

As it stands, the environment described above is pretty pointless since there's no way to control any of the models or take any observations from them. In fact if you print the action/observation spaces for the environment at this point they'll just be empty dictionaries.

To actually interact with an environment we use entities called add-ons. An add-on is a chunk of code that can be attached to a model or the environment itself which interacts with the simulation whenever the user calls either `step` or `reset` on the RoboGym environment.

There's a slew of built-in add-ons that can be used to control robot arm end effectors, attach cameras to models and so forth so there's already a pretty decent set of environments that can be defined out-of-the-box. You can also define your own add-ons relatively easily, see below for more.

To see how the built-in add-ons work, let's add a couple of few to the jaco environment:

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

    <model name="r2d2">
        <xyz>0.0 0.5 0.7</xyz>
        <scale>0.1</scale>
        <urdf>r2d2.urdf</urdf>

        <addon name="r2d2" type="camera">
            <base_frame>left_tip_joint</base_frame>
            <resolution>200 200</resolution>
        </addon>

    </model>

    <model name="robot">
        <urdf>jaco/j2s7s300_standalone.urdf</urdf>
        <xyz>0.0 0.0 0.65</xyz>

        <addon name="controller" type="position_controller">
            <rest_position>0.0 2.9 0.0 1.3 4.2 1.4 0.0 1.0 1.0 1.0</rest_position>
            <end_effector_frame>j2s7s300_joint_end_effector</end_effector_frame>
        </addon>

        <addon name="be_lazy" type="electricity_cost"/>

    </model>
</environment>
```
These additions will do the following:
* `r2d2_camera` will take RGB and depth images from the left_tip_joint frame of R2D2 and add those observations to the dict returned when RoboGym is `step`ped or `reset`
* `controller` will move the end effector of the jaco arm in cartesian mode in response actions passed to `step`
* `be_lazy` will calculate a penalty for the amount of joint torque currently being applied by the jaco and this penalty will be included in the reward value returned by `step`

### Writing Your Own Addons:

If you need to customise your environment beyond what's possible with the built-in addons it's pretty easy to add your own. To do so, just subclass `Addon` in addons/addon.py and follow the instructions in the docstrings to fill out your desired callbacks.

Once you're satisfied with your addon you can register it with RoboGym using the `AddonFactory` class like so:
```
from robo_gym.addons.addon import Addon, AddonFactory

class MyAddon(Addon):
    ...

AddonFactory.register_addon('my_addon', MyAddon)
```
This will add your addon to a dictionary maintained by the factory so choose a name that doesn't clash with any of built-in addons or any others that you've defined.

Once the addon is added you can refer to it in a config file just as you would any other addon and use that file to create a RoboGym:
```
env = RoboGym(path_to_config_file)
```
For an actual example of how to add a addon to RoboGym, check out the [jaco_on_a_table](https://github.com/ascentai/robo-gym/tree/master/examples/jaco_on_a_table) example.
