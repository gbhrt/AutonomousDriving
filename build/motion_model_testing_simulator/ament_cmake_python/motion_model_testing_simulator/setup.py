import os
from setuptools import find_packages
from setuptools import setup

setup(
    name='motion_model_testing_simulator',
    version='0.0.1',
    packages=find_packages(
        include=('motion_model_testing_simulator', 'motion_model_testing_simulator.*')),
)
