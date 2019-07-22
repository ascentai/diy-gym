from setuptools import setup

setup(name='diy_gym',
      version='0.1',
      description='A framework for building your own reinforcement learning environments',
      url='http://github.com/ascentai/diy_gym',
      author='AscentAI',
      author_email='opensource@ascent.ai',
      license='MIT',
      classifiers=['License :: OSI Approved :: MIT License', 'Programming Language :: Python :: 3'],
      python_requires='>=3',
      packages=['diy_gym'],
      package_data={'diy_gym': ['data/*']},
      zip_safe=False)
