#!/usr/bin/env python3
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=['garbage_quick_sort_robot_ros'],
    package_dir={'': 'src'},
    install_requires=['numpy', 'roboticstoolbox']
)

setup(**setup_args)