from setuptools import find_packages
from setuptools import setup

setup(
    name='ros2_kdl_package',
    version='0.0.0',
    packages=find_packages(
        include=('ros2_kdl_package', 'ros2_kdl_package.*')),
)
