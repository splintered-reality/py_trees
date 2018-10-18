#!/usr/bin/env python

from setuptools import find_packages, setup
import os

# You need install_requires if you don't have a ROS environment
install_requires = [ # ] if os.environ.get('AMENT_PREFIX_PATH') else [
    # build
    'setuptools',
    # runtime
    'enum34;python_version<"3.4"',
    'pydot'
]

tests_require=['nose', 'pytest', 'flake8', 'yanc', 'nose-htmloutput']

extras_require = {} if os.environ.get('AMENT_PREFIX_PATH') else {
    'test': tests_require,
    'docs': ["Sphinx", "sphinx-argparse", "sphinx_rtd_theme"],
    'debs': ['stdeb', 'twine']
}
##############################
# Pull in __version__
##############################

# Can't use __file__ of setup.py to determine
# the relative path to ./py_trees/version.py since
# ament doesn't actually use this file - it parses
# this file and executes it directly.


# Some duplication of properties here and in package.xml.
# Make sure to update them both.
# That is the price paid for a pypi and catkin package.
d = setup(
    name='py_trees',
    version='0.8.1',  # also update package.xml and version.py
    packages=find_packages(exclude=['tests*', 'docs*']),
    install_requires=install_requires,
    extras_require=extras_require,
    author='Daniel Stonier, Naveed Usmani, Michal Staniaszek',
    maintainer='Daniel Stonier <d.stonier@gmail.com>',
    url='http://github.com/stonier/py_trees',
    keywords='behaviour-trees',
    zip_safe=True,
    classifiers=[
        'Environment :: Console',
        'Intended Audience :: Developers',
        'License :: OSI Approved :: BSD License',
        'Programming Language :: Python',
        'Topic :: Scientific/Engineering :: Artificial Intelligence',
        'Topic :: Software Development :: Libraries'
    ],
    description="pythonic implementation of behaviour trees",
    long_description="A behaviour tree implementation for rapid development of small scale decision making engines that don't need to be real time reactive.",
    license='BSD',
    test_suite='nose.collector',
    tests_require=tests_require,
    entry_points={
         'console_scripts': [
             'py-trees-demo-action-behaviour = py_trees.demos.action:main',
             'py-trees-demo-behaviour-lifecycle = py_trees.demos.lifecycle:main',
             'py-trees-demo-blackboard = py_trees.demos.blackboard:main',
             'py-trees-demo-context-switching = py_trees.demos.context_switching:main',
             'py-trees-demo-dot-graphs = py_trees.demos.dot_graphs:main',
             'py-trees-demo-pick-up-where-you-left-off = py_trees.demos.pick_up_where_you_left_off:main',
             'py-trees-demo-selector = py_trees.demos.selector:main',
             'py-trees-demo-sequence = py_trees.demos.sequence:main',
             'py-trees-demo-tree-stewardship = py_trees.demos.stewardship:main',
         ],
     },
)
