from setuptools import setup

setup(
    name='robo_gym',
    version='0.1',
    description='A flexible simulation environment for robot arms to drive RL research @ Ascent',
    url='http://github.com/ascentai/robo_gym',
    author='AscentAI',
    author_email='dev@ascent.ai',
    license='MIT',
    classifiers=[
        'License :: OSI Approved :: MIT License',
        'Programming Language :: Python :: 3'],
    python_requires='>=3',
    packages=['robo_gym'],
    package_data={'robo_gym': ['data/*']},
    zip_safe=False
)
