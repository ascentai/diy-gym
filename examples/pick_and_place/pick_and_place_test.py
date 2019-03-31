#!/usr/bin/env python
import argparse
import os
import time
import torch
import numpy as np
import matplotlib.pyplot as plt

from robo_gym import RoboGym
from TD3.utils import ReplayBuffer, evaluate_policy
from TD3.TD3 import TD3

if __name__ == '__main__':

    np.random.seed(101)

    parser = argparse.ArgumentParser()
    parser.add_argument("--seed", default=0, type=int)  # Sets Gym, PyTorch and Numpy seeds
    parser.add_argument("--start_timesteps", default=1e4,
                        type=int)  # How many time steps purely random policy is run for
    parser.add_argument("--eval_freq", default=5e3, type=float)  # How often (time steps) we evaluate
    parser.add_argument("--max_timesteps", default=1e6, type=float)  # Max time steps to run environment for
    parser.add_argument("--save_models", action="store_true")  # Whether or not models are saved
    parser.add_argument("--expl_noise", default=0.1, type=float)  # Std of Gaussian exploration noise
    parser.add_argument("--batch_size", default=100, type=int)  # Batch size for both actor and critic
    parser.add_argument("--discount", default=0.99, type=float)  # Discount factor
    parser.add_argument("--tau", default=0.005, type=float)  # Target network update rate
    parser.add_argument("--policy_noise", default=0.2, type=float)  # Noise added to target policy during critic update
    parser.add_argument("--noise_clip", default=0.5, type=float)  # Range to clip target policy noise
    parser.add_argument("--policy_freq", default=2, type=int)  # Frequency of delayed policy updates
    args = parser.parse_args()

    file_name = "%s_%s_%s" % ('TD3', 'pick_and_place', str(args.seed))
    print("---------------------------------------")
    print("Settings: %s" % (file_name))
    print("---------------------------------------")

    env = RoboGym('pick_and_place.yaml')

    # Set seeds
    env.seed(args.seed)
    torch.manual_seed(args.seed)
    np.random.seed(args.seed)

    state_dim = env.observation_space.shape[0]
    action_dim = env.action_space.shape[0]
    max_action = float(env.action_space.high[0])

    # Initialize policy
    policy = TD3(state_dim, action_dim, max_action)

    # Load policy
    policy.load("%s" % file_name, directory="./pytorch_models")

    # Evaluate trained policy
    evaluations = [evaluate_policy(env, policy)]

    total_timesteps = 0
    timesteps_since_eval = 0
    episode_num = 0
    done = True

    while total_timesteps < args.max_timesteps:

        if done:
            if total_timesteps != 0:
                print("Total T: %d Episode Num: %d Episode T: %d Reward: %f"
                      % (total_timesteps, episode_num, episode_timesteps, episode_reward))

            # Evaluate episode
            if timesteps_since_eval >= args.eval_freq:
                timesteps_since_eval %= args.eval_freq
                evaluations.append(evaluate_policy(env, policy))

            # Reset environment
            obs = env.reset()
            done = False
            episode_reward = 0
            episode_timesteps = 0
            episode_num += 1

        action = policy.select_action(np.array(obs))

        # Perform action
        new_obs, reward, done, _ = env.step(action)
        done_bool = 0 if episode_timesteps + 1 == env._max_episode_steps else float(done)
        episode_reward += reward

        obs = new_obs

        episode_timesteps += 1
        total_timesteps += 1
        timesteps_since_eval += 1

