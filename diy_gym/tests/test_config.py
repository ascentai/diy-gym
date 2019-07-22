import unittest
import os
from diy_gym.config import Configuration


class TestConfiguration(unittest.TestCase):
    def setUp(self):
        config_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'basic_env.yaml')
        self.config = Configuration.from_file(config_file)

    def test_get_config(self):
        self.assertTrue(self.config.get('im_a_config'))

    def test_default_config(self):
        self.assertFalse(self.config.get('im_not_a_config', False))

    def test_set_config(self):
        self.config.set('im_a_config_now_too', 5.0)
        self.assertEqual(self.config.get('im_a_config_now_too'), 5.0)

    def test_find_all(self):
        models = self.config.find_all('model')
        self.assertEqual(len(list(models)), 4)


if __name__ == '__main__':
    unittest.main()
