#!/usr/bin/env python

from setuptools import find_packages, setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=find_packages(exclude=['tests*', 'docs*']),
    # package_data = {'rocon_uri': ['rules/rules.yaml']},
    # global installs, for package relative installs, look in CMakeLists.txt
    # scripts=['scripts/demo_tree']
)

setup(**d)
