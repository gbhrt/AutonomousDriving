import os
from setuptools import find_packages
from setuptools import setup

setup(
    name='robot_steering_converter',
    version='0.0.1',
    packages=find_packages(
        include=('robot_steering_converter', 'robot_steering_converter.*')),
)
