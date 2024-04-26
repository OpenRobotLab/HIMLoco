from setuptools import find_packages
from distutils.core import setup

setup(
    name='legged_gym',
    version='1.0.0',
    author='Junfeng Long, Zirui Wang, Nikita Rudin',
    license="BSD-3-Clause",
    packages=find_packages(),
    author_email='',
    description='Isaac Gym environments for Legged Robots',
    install_requires=['isaacgym',
                      'rsl-rl',
                      'matplotlib']
)