import yaml
import os
import numpy as np
import math as m
import random
import argparse

if __name__ == '__main__':

    parser = argparse.ArgumentParser(description='Generate a maze for an R2D2 to drive around in')
    parser.add_argument('--maze_size', default=10, type=int, help='the dimensions of the maze to be generated')
    parser.add_argument('--use_gripper_camera',
                        type=lambda x: bool(strtobool(x)),
                        default=False,
                        help='whether to attach a camera to the R2D2s gripper')
    args = parser.parse_args()

    # Make a dictionary containing the usual configs along with an R2D2 for us to drive around the maze
    env_config = {
        'camera_yaw': 0,
        'camera_distance': 12,
        'r2d2': {
            'model': 'r2d2.urdf',
            'xyz': [1.5 - args.maze_size / 2, -0.5 - args.maze_size / 2, 0.5],
            'mass': 50,
            'wheel_driver': {
                'addon':
                'joint_controller',
                'joints': [
                    'left_front_wheel_joint', 'left_back_wheel_joint', 'right_front_wheel_joint',
                    'right_back_wheel_joint'
                ],
                'control_mode':
                'velocity'
            },
            'respawn': {
                'addon': 'respawn'
            }
        },
        'plane': {
            'model': 'grass/plane.urdf'
        }
    }

    # Optionally we can also attach a camera to the gripper of the R2D2 to see what it sees as it drives around
    if args.use_gripper_camera:
        env_config['r2d2']['gripper_camera'] = {
            'addon': 'camera',
            'frame': 'left_gripper_joint',
            'xyz': [0, 0, -0.1],
            'rpy': [m.pi / 2, 0, -m.pi / 2],
            'resolution': [100, 100]
        }

    wall_config = {'model': 'wall/wall.urdf', 'use_fixed_base': True, 'scale': 0.01}

    # Now we actually generate a maze. This is done using a random Depth First Search
    # source: https://en.wikipedia.org/wiki/Maze_generation_algorithm#Depth-first_search
    visited = np.zeros(shape=(args.maze_size, args.maze_size), dtype=np.bool)
    offsets = [np.array(offset) for offset in [(0, 1), (1, 0), (0, -1), (-1, 0)]]

    path = [(0, 1)]
    maze = {path[-1]: [(-1, 1)]}

    while len(path):
        unvisited_neighbours = [
            tuple(path[-1] + offset) for offset in offsets
            if not tuple((path[-1] + offset).clip(0, args.maze_size - 1)) in maze
        ]

        if len(unvisited_neighbours):
            next_cell = random.choice(unvisited_neighbours)
            maze[next_cell] = [path[-1]]
            maze[path[-1]].append(next_cell)
            path.append(next_cell)
        else:
            path.pop()

    maze[(args.maze_size - 1, args.maze_size - 2)].append((args.maze_size, args.maze_size - 2))

    # The maze is now expressed as a bunch of cells with a list of neighbours, we'll now just create walls to
    # separate adjacent cells that aren't neighbours
    walls = []
    for cell, edges in maze.items():
        for i, offset in enumerate(offsets):
            if tuple(np.add(cell, offset)) in edges:
                continue

            coords = offset * 0.5 + cell - args.maze_size / 2 + 0.5
            walls.append((float(coords[1]), float(coords[0]), float(m.pi / 2 * abs(offset[1]))))

    # now remove all the duplicate walls by converting to a dictionary and back again (this feels a bit hacky but google tells me it's ok)
    walls = list(dict.fromkeys(walls))

    for i, (x, y, theta) in enumerate(walls):
        new_wall = wall_config.copy()
        new_wall['xyz'] = [x, y, 0.0]
        new_wall['rpy'] = [m.pi / 2, 0, theta]
        env_config['wall_%d' % i] = new_wall

    # The environment is now completely described by env_config, we'll dump this to a yaml file so we
    # can pass it to DIYGym later
    config_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'r2d2_maze.yaml')

    with open(config_file, 'w') as f:
        f.write(yaml.dump(env_config))
