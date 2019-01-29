import os
import glob
import shutil
import random
import tarfile
import requests
import pybullet as p

from tqdm import tqdm
from robo_gym.addons.misc.randomizer_interface import RandomizerInterface


class VisualRandomizer(RandomizerInterface):
    def __init__(self, parent, config):
        super(VisualRandomizer, self).__init__(parent, config)

        self.data_dir = os.path.normpath(os.path.dirname(os.path.abspath(__file__)) + '/../../data')
        self.texture_dir = os.path.normpath(self.data_dir + '/textures')
        self.randomize_color = config.get('randomize_color')
        self.reset()

    def reset(self):
        for joint_id in self.joint_ids:
            if self.randomize_color:
                p.changeVisualShape(self.uid, joint_id, rgbaColor=self.random_color_generator())
            else:
                try:
                    random_texture = random.choice(os.listdir(self.texture_dir))
                    p.changeVisualShape(self.uid, joint_id,
                                        textureUniqueId=p.loadTexture(os.path.join(self.texture_dir, random_texture)))
                except FileNotFoundError:
                    self.get_dataset()

    def get_dataset(self):
        print('robo_gym/data/textures folder not found, downloading dataset...')
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
    def random_color_generator():
        rgba_color = []
        for i in range(4):
            rgba_color.append(random.random())
        return rgba_color

    @staticmethod
    def download_dataset(url, filename):
        """
        Helper method handling downloading large files from `url` to `filename`. Returns a pointer to `filename`.
        """
        chunkSize = 1024
        r = requests.get(url, stream=True)
        with open(filename, 'wb') as f:
            pbar = tqdm(unit="B", total=int(r.headers['Content-Length']))
            for chunk in r.iter_content(chunk_size=chunkSize):
                if chunk:  # filter out keep-alive new chunks
                    pbar.update(len(chunk))
                    f.write(chunk)
        return filename
