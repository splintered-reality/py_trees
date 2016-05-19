#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['py_trees'],
    package_dir={'': 'src'},
    #package_data = {'rocon_uri': ['rules/rules.yaml']},
    scripts=['scripts/blackboard_watcher'],
    #scripts=['scripts/rocon_uri'],
)

setup(**d)
