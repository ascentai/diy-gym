import os
import glob
import shutil
import random
import tarfile
import requests
import pybullet as p
import numpy as np

from tqdm import tqdm
from diy_gym.addons.addon import Addon


class VisualRandomizer(Addon):
    max_textures = 500
    loaded_textures = []

    def __init__(self, parent, config):
        super(VisualRandomizer, self).__init__(parent, config)

        self.uid = parent.uid

        self.joint_ids = [
            p.getJointInfo(self.uid, i)[0] for i in range(p.getNumJoints(self.uid))
            if p.getJointInfo(self.uid, i)[3] > -1
        ]
        self.joint_ids = [-1] if not self.joint_ids else self.joint_ids

        self.data_dir = os.path.normpath(os.path.dirname(os.path.abspath(__file__)) + '/../../data')
        self.texture_dir = os.path.normpath(self.data_dir + '/textures')
        self.reset()

    def get_texture(self):
        if len(VisualRandomizer.loaded_textures) < VisualRandomizer.max_textures:
            random_texture = random.choice(os.listdir(self.texture_dir))
            VisualRandomizer.loaded_textures.append(p.loadTexture(os.path.join(self.texture_dir, random_texture)))
            return VisualRandomizer.loaded_textures[-1]
        else:
            return random.choice(VisualRandomizer.loaded_textures)

    def reset(self):
        for joint_id in self.joint_ids:
            try:
                p.changeVisualShape(self.uid, joint_id, textureUniqueId=self.get_texture())
            except FileNotFoundError:
                self.get_dataset()

    def get_dataset(self):
        print('diy_gym/data/textures folder not found, downloading dataset...')
        self.download_dataset('https://www.robots.ox.ac.uk/~vgg/data/dtd/download/dtd-r1.0.1.tar.gz',
                              self.data_dir + '/textures.tar.gz')

        print('\nDownload complete, extracting dataset...')
        tf = tarfile.open(self.data_dir + '/textures.tar.gz')
        tf.extractall(path=self.data_dir)
        print('Extraction complete...')

        os.mkdir(self.texture_dir)

        for texture_path in tqdm(glob.glob(self.data_dir + '/dtd/images/**/*.jpg', recursive=True)):
            shutil.move(texture_path, self.texture_dir)

        # Cleanup
        shutil.rmtree(self.data_dir + '/dtd')
        os.remove(self.data_dir + '/textures.tar.gz')

    @staticmethod
    def download_dataset(url, filename):
        """
        Helper method handling downloading large files from `url` to `filename`. Returns a pointer to `filename`.
        """
        r = requests.get(url, stream=True)
        with open(filename, 'wb') as f:
            for chunk in tqdm(r.iter_content(chunk_size=1024), unit="B", total=int(r.headers['Content-Length']) / 1024):
                if chunk:  # filter out keep-alive new chunks
                    f.write(chunk)
        return filename
