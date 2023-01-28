#!/usr/bin/env python

################################################################################
# This is a minimal setup.py for enabling ROS builds.
#
# For all other modes of development, use poetry and pyproject.toml
################################################################################

import os

from setuptools import find_packages, setup

install_requires = ["setuptools", "pydot"]

# Some duplication of properties in:
#  - setup.py,           (ros / legacy)
#  - package.xml         (ros)
#  - pyproject.toml      (poetry)
#  - py_trees/version.py (common)
# Keep them in sync.
d = setup(
    name="py_trees",
    version="2.2.1",
    packages=find_packages(exclude=["tests*", "docs*"]),
    package_data={"py_trees": ["py.typed"]},
    install_requires=install_requires,
    author="Daniel Stonier, Naveed Usmani, Michal Staniaszek",
    maintainer="Daniel Stonier <d.stonier@gmail.com>",
    url="https://github.com/splintered-reality/py_trees",
    keywords="behaviour-trees",
    zip_safe=True,
    classifiers=[
        "Environment :: Console",
        "Intended Audience :: Developers",
        "License :: OSI Approved :: BSD License",
        "Programming Language :: Python",
        "Topic :: Scientific/Engineering :: Artificial Intelligence",
        "Topic :: Software Development :: Libraries",
    ],
    description="pythonic implementation of behaviour trees",
    long_description="A behaviour tree implementation for rapid development of small scale decision making engines that don't need to be real time reactive.",
    license="BSD",
    entry_points={
        "console_scripts": [
            "py-trees-render = py_trees.programs.render:main",
            "py-trees-demo-action-behaviour = py_trees.demos.action:main",
            "py-trees-demo-behaviour-lifecycle = py_trees.demos.lifecycle:main",
            "py-trees-demo-blackboard = py_trees.demos.blackboard:main",
            "py-trees-demo-blackboard-namespaces = py_trees.demos.blackboard_namespaces:main",
            "py-trees-demo-blackboard-remappings = py_trees.demos.blackboard_remappings:main",
            "py-trees-demo-context-switching = py_trees.demos.context_switching:main",
            "py-trees-demo-display-modes = py_trees.demos.display_modes:main",
            "py-trees-demo-dot-graphs = py_trees.demos.dot_graphs:main",
            "py-trees-demo-either-or = py_trees.demos.either_or:main",
            "py-trees-demo-eternal-guard = py_trees.demos.eternal_guard:main",
            "py-trees-demo-logging = py_trees.demos.logging:main",
            "py-trees-demo-pick-up-where-you-left-off = py_trees.demos.pick_up_where_you_left_off:main",
            "py-trees-demo-selector = py_trees.demos.selector:main",
            "py-trees-demo-sequence = py_trees.demos.sequence:main",
            "py-trees-demo-tree-stewardship = py_trees.demos.stewardship:main",
        ],
    },
)
