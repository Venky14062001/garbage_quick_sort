#!/usr/bin/env python3
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=['detector'],
    package_dir={'': 'src'},
    install_requires=['cv2', 'torch', 'numpy', 'math', 'yolov5']
)

setup(**setup_args)