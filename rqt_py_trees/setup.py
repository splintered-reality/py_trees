#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['rqt_py_trees', 'rqt_py_trees.plugins', 'rqt_py_trees.qt_dotgraph'],
    package_dir={'': 'src'},
    scripts=['scripts/rqt_py_trees']
)

setup(**d)
