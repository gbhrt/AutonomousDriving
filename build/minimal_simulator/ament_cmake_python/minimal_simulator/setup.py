import os
from setuptools import find_packages
from setuptools import setup

setup(
    name='minimal_simulator',
    version='0.0.1',
    packages=find_packages(
        include=('minimal_simulator', 'minimal_simulator.*')),
)
