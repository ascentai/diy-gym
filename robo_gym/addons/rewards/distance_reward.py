import numpy as np

from ..addon import Addon
from ..sensors.distance_sensor import DistanceSensor

'''
reference [https://github.com/ascentai/yamaha_demo/tree/new-demo-testing]

Negatively proportional to the delta (since the last time step) of the Euclidean distance between the location of the 
end-of-effector and a "target location" for grasping (the point 25cm above the block). This reward component is a 
hint that encourages the agent to approach to the block. Intuitively, if the gripper moves closer (in the last time 
step), there will be a negative delta of distance, which leads to a positive gripper-movement reward; otherwise, 
moving far away corresponds to a positive delta of distance, and thus a negative gripper-movement reward. 

Ideally, a movement of the gripper along the direction straightly towards the target location maximizes the 
gripper-movement reward. However, the agent cannot perfectly accurately follow this optimal velocity because the 
current velocity of the gripper (which the control needs to compensate) is unknown to the agent, and possibly also 
because the agent output cannot directly control the velocity at all (as mentioned above, the actual effect of the 
agent output on robot arm movement is unclear). Nevertheless, experimental results show that there is still a 
correlation between the value of gripper-movement reward obtained and the direction of the target position of the 
gripper (i.e. action[0-2]) – as shown in the chart below, statistically speaking, the more the target movement aligns 
to the optimal velocity direction (measured by the cosine of the angle between the vectors), the higher the 
gripper-movement reward is. It is probably this statistical correlation that guides the robot arm towards the block 
under the gripper-movement rewards.'''


class DistanceReward(Addon):
    def __init__(self, parent, config):
        super(DistanceReward, self).__init__(parent, config)

        self.dist_sensor  = DistanceSensor(parent, config)
        self.initial_dist = self.dist_sensor.observe()['distance']
        self.prev_dist    = self.initial_dist

        self.multiplier = config.get('multiplier', 100)
        self.tolerance  = config.get('tolerance' , 0.05)
        self.done = False

    def reward(self):
        # get updated observations
        curr_dist = self.dist_sensor.observe()['distance']
        delta_dist = self.prev_dist - curr_dist

        if curr_dist <= self.tolerance:
            self.done = True
            return 1000

        # update prev_dist with the curr_dist
        self.prev_dist = curr_dist

        # return np.exp(-curr_dist)
        return self.multiplier * delta_dist

    def reset(self):
        self.prev_dist = self.initial_dist
        self.done = False

    def is_terminal(self):
        return self.done


