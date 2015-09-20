#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['gopher_behaviours'],
    package_dir={'': 'src'},
    #package_data = {'rocon_uri': ['rules/rules.yaml']},
    scripts=['scripts/dslam_maps'],
)

setup(**d)
