import os
from setuptools import find_packages
from setuptools import setup

setup(
    name='simulation_sync',
    version='0.0.1',
    packages=find_packages(
        include=('simulation_sync', 'simulation_sync.*')),
)
