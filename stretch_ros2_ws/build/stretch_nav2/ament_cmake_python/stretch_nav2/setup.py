from setuptools import find_packages
from setuptools import setup

setup(
    name='stretch_nav2',
    version='0.0.1',
    packages=find_packages(
        include=('stretch_nav2', 'stretch_nav2.*')),
)
