import os
import glob
import random
import shutil
import tarfile
import pybullet as p

import requests
from tqdm import tqdm

from ..addon import Addon


def random_color_generator():
    rgba_color = []
    for i in range(4):
        rgba_color.append(random.random())
    return rgba_color


def download_file(url, filename):
    """
    Helper method handling downloading large files from `url` to `filename`. Returns a pointer to `filename`.
    """
    chunkSize = 1024
    r = requests.get(url, stream=True)
    with open(filename, 'wb') as f:
        pbar = tqdm( unit="B", total=int( r.headers['Content-Length'] ) )
        for chunk in r.iter_content(chunk_size=chunkSize):
            if chunk: # filter out keep-alive new chunks
                pbar.update (len(chunk))
                f.write(chunk)
    return filename


class DomainRandomization(Addon):
    def __init__(self, parent, config):
        super(DomainRandomization, self).__init__()

        self.uid = parent.uid

        self.joint_ids = [p.getJointInfo(self.uid, i)[0] for i in range(p.getNumJoints(self.uid))]
        self.texture_dir = '../../robo_gym/data/textures'
        self.randomize_color = config.get('randomize_color')

        self.joint_ids = [-1] if not self.joint_ids else self.joint_ids
        self.reset()

    def reset(self):
        self.randomize_visuals()
        self.randomize_dynamics()

    def randomize_visuals(self):

        for joint_id in self.joint_ids:
            if self.randomize_color:
                p.changeVisualShape(self.uid, joint_id, rgbaColor=random_color_generator())
            else:
                try:
                    random_texture = random.choice(os.listdir(self.texture_dir))
                    p.changeVisualShape(self.uid, joint_id,
                                        textureUniqueId=p.loadTexture(os.path.join(self.texture_dir, random_texture)))
                except FileNotFoundError:
                    self.get_dataset()

    def randomize_dynamics(self):
        # reference [https://arxiv.org/pdf/1710.06537.pdf] Table 1 for parameter ranges
        mass_multiplier = random.uniform(0.25, 4.0)
        joint_damping_multiplier = random.uniform(0.2, 20)

        for joint_id in self.joint_ids:
            new_mass_value = mass_multiplier * p.getDynamicsInfo(self.uid, joint_id)[0]
            new_joint_damping_value = joint_damping_multiplier * 0.4

            p.changeDynamics(self.uid, joint_id, mass=new_mass_value, angularDamping=new_joint_damping_value)

    def get_dataset(self):

        print('robo_gym/data/textures folder not found, downloading dataset...')
        download_file('https://www.robots.ox.ac.uk/~vgg/data/dtd/download/dtd-r1.0.1.tar.gz',
                      os.path.join(os.getcwd(), '../../robo_gym/data/textures.tar.gz'))

        print('\nDownload complete, extracting dataset...')
        tf = tarfile.open('../../robo_gym/data/textures.tar.gz')
        tf.extractall(path='../../robo_gym/data/')
        print('Extraction complete...')

        os.mkdir(self.texture_dir)

        for texture_path in tqdm(glob.glob('../../robo_gym/data/dtd/images/**/*.jpg', recursive=True)):
            shutil.move(texture_path, self.texture_dir)

        # Cleanup
        shutil.rmtree('../../robo_gym/data/dtd')
        os.remove('../../robo_gym/data/textures.tar.gz')
