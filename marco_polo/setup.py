#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['marco_polo'],
    package_dir={'': 'src'},
    # package_data = {'rocon_uri': ['rules/rules.yaml']},
    scripts=['scripts/gopher_teleport',
             'scripts/gopher_maps'],
)

setup(**d)
