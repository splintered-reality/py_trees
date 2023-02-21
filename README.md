# Py Trees

[[About](#about)] [[What's New?](#whats-new)] [[Documentation](#documentation)] [[Getting Started](#getting-started)] [[Next Steps](#next-steps)] [[Releases](#releases)]

----

## About

PyTrees is a python implementation of behaviour trees designed to facilitate the rapid development of medium sized decision making engines for use in fields like robotics. Brief feature list:

* Behaviours, Decorators, Sequences, Selectors, Parallels and BehaviourTree.
* Blackboards for data sharing.
* A useful library of behaviours, decorators and idioms.
* Serialise to a dot graph or render to ascii/unicode in a terminal.
* Tested on Linux and Mac (YMMV with Windows).

## What's New?

* [2023-01-28] The [2.2.x Release][new-2.2.x] is out! Sequences and Selectors with AND without memory.
* [2023-01-28] [Get Started](#getting-started) with py_trees in under 5mins.
* [2023-01-28] This is now a `poetry` project with configuration governed by `pyproject.toml`.
  * The vestigial `setup.py` is only to assist distribution where [PEP-517](https://peps.python.org/pep-0517/) is not yet well supported.

## Documentation

[![devel][docs-devel-image]][docs-devel] [![2.2.x][docs-2.2.x-image]][docs-2.2.x] [![2.1.x][docs-2.1.x-image]][docs-2.1.x] [![0.7.x][docs-0.7.x-image]][docs-0.7.x] [![0.6.x][docs-0.6.x-image]][docs-0.6.x]

## Getting Started

You can get started on CodeSpaces (with no mismatched environment issues and in under 5mins) [1]:

1. Fork the project to your personal account
2. Click on Code -> Codespaces -> + Create a Codespace
3. Enter the Terminal

```
# Install Dependencies
(docker) zen@py_trees:/workspaces/py_trees$ poetry install

# Explore the demos
(docker) zen@py_trees:/workspaces/py_trees$ poetry shell
(py-trees-py3.8) (docker) zen@py_trees:/workspaces/py_trees$ py-trees-demo-<tab>-<tab>
py-trees-demo-action-behaviour            py-trees-demo-context-switching           py-trees-demo-logging
py-trees-demo-behaviour-lifecycle         py-trees-demo-display-modes               py-trees-demo-pick-up-where-you-left-off
py-trees-demo-blackboard                  py-trees-demo-dot-graphs                  py-trees-demo-selector
py-trees-demo-blackboard-namespaces       py-trees-demo-either-or                   py-trees-demo-sequence
py-trees-demo-blackboard-remappings       py-trees-demo-eternal-guard               py-trees-demo-tree-stewardship
(py-trees-py3.8) (docker) zen@py_trees:/workspaces/py_trees$ py-trees-demo-blackboard
...
(py-trees-py3.8) (docker) zen@py_trees:/workspaces/py_trees$ exit

# Hack some Code

# Run the Formatter, Tests, Linters and Mypy
(docker) zen@py_trees:/workspaces/py_trees$ poetry run tox -l
py38 py310 format check mypy38 mypy310
(docker) zen@py_trees:/workspaces/py_trees$ poetry run tox -e format
...
(docker) zen@py_trees:/workspaces/py_trees$ poetry run tox -e py38
...
(docker) zen@py_trees:/workspaces/py_trees$ poetry run tox -e check
...

# Contribute a PR!
# https://github.com/splintered-reality/py_trees/blob/devel/CONTRIBUTING.md
```

[1] All of the above will, of course, work in a local environment if you have `poetry` installed. If you're using `VSCode`
you don't even need that, just reopen the project in the [devcontainer](.devcontainer/devcontainer.json) and be froody.

## Next Steps

On PyPi:
* [py_trees](https://pypi.org/project/py-trees/)
* [py_trees_js](https://pypi.org/project/py-trees-js/)

Examples:
* [ReadTheDocs - PyTrees ROS Tutorials](https://py-trees-ros-tutorials.readthedocs.io/en/release-2.0.x/index.html) - significantly more edifying than the demos, these incrementally walk through the process of building a decision making layer for a robot. These use ROS2 (sparsely), but merely browsing should be enlightening regardless.

Visualisation:
* [py_trees_js](https://github.com/splintered-reality/py_trees_js) - a javascript library for building your own runtime visualisation tool

Robotics:
* [py_trees_ros](https://github.com/splintered-reality/py_trees_ros) - a tree manager and behaviours designed for use specifically with `ROS2`
* [py_trees_ros_viewer](https://github.com/splintered-reality/py_trees_ros_viewer) - a `Qt/ROS2` implementation of `py_trees_js`

## Releases

* `2.2.x` - Selectors, Sequences with and without memory. Improved testing and style/type checking.
* `2.1.x` - Chooser deprecated. API housekeeping.
* `2.0.x` - Blackboards V2!
* `1.2.x` - Trees can now shutdown cleanly. StatusToBlackboard and EternalGuard, Visitors get finalise().
* `1.1.x` - Fixes for setup, tick-tock, viz.
* `1.0.x` - Behaviours, Decorators, Composites, Blackboards, Tree Management and Viz tools.
* `0.y.x` - First open source pre-releases.

| | Devel | 2.2.x | 2.1.x | 2.0.x | 1.2.x | 0.7.x | 0.6.x |
|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
| Sources | [![devel][sources-devel-image]][sources-devel] | [![2.1.x][sources-2.2.x-image]][sources-2.2.x] | [![2.1.x][sources-2.1.x-image]][sources-2.1.x] | [![2.0.x][sources-2.0.x-image]][sources-2.0.x] | [![1.2.x][sources-1.2.x-image]][sources-1.2.x] | [![0.7.x][sources-0.7.x-image]][sources-0.7.x] | [![0.6.x][sources-0.6.x-image]][sources-0.6.x]
| Compatibility | [![Python 3.10][python310-image]][python310-docs] [![Python 3.8][python38-image]][python38-docs] | [![Python 3.10][python310-image]][python310-docs] [![Python 3.8][python38-image]][python38-docs] | [![Python 3.6][python36-image]][python36-docs] | [![Python 3.6][python36-image]][python36-docs] | [![Python 3.6][python36-image]][python36-docs] | [![Python 3.6][python36-image]][python36-docs] | [![Python 2.7][python27-image]][python27-docs]
| CI | [![devel-Status][devel-build-status-image]][devel-build-status] | [![2.2.x-Status][2.2.x-build-status-image]][2.2.x-build-status] | - | - | - | - | - | 
| Documentation | [![devel-Docs][rtd-devel-image]][docs-devel] | [![2.2.x-Docs][rtd-2.2.x-image]][docs-2.2.x] | [![2.1.x-Docs][rtd-2.1.x-image]][docs-2.1.x] | [![2.0.x-Docs][rtd-2.0.x-image]][docs-2.0.x] | [![1.2.x-Docs][rtd-1.2.x-image]][docs-1.2.x] | [![0.7.x-Docs][rtd-0.7.x-image]][docs-0.7.x] | [![0.6.x-Docs][rtd-0.6.x-image]][docs-0.6.x]


[license-image]: https://img.shields.io/badge/License-BSD%203--Clause-orange.svg?style=plastic
[license]: LICENSE

[python310-image]: https://img.shields.io/badge/python-3.10-green.svg?style=plastic
[python310-docs]: https://docs.python.org/3.10/
[python38-image]: https://img.shields.io/badge/python-3.8-green.svg?style=plastic
[python38-docs]: https://docs.python.org/3.8/
[python36-image]: https://img.shields.io/badge/python-3.6-green.svg?style=plastic
[python36-docs]: https://docs.python.org/3.6/
[python27-image]: https://img.shields.io/badge/python-2.7-green.svg?style=plastic
[python27-docs]: https://docs.python.org/2.7/

[devel-build-status-image]: https://github.com/splintered-reality/py_trees/actions/workflows/pre-merge.yaml/badge.svg
[devel-build-status]: https://github.com/splintered-reality/py_trees/actions/workflows/pre-merge.yaml
[2.2.x-build-status-image]: https://github.com/splintered-reality/py_trees/actions/workflows/pre-merge.yaml/badge.svg??branch=release/2.2.x
[2.2.x-build-status]: https://github.com/splintered-reality/py_trees/actions/workflows/pre-merge.yaml

[new-2.2.x]: https://github.com/splintered-reality/py_trees/blob/devel/CHANGELOG.rst#22x-2023-01-23---sequences-and-selectors-with-and-without-memory


[docs-devel]: http://py-trees.readthedocs.io/
[docs-2.2.x]: http://py-trees.readthedocs.io/en/release-2.2.x/
[docs-2.1.x]: http://py-trees.readthedocs.io/en/release-2.1.x/
[docs-2.0.x]: http://py-trees.readthedocs.io/en/release-2.0.x/
[docs-1.3.x]: http://py-trees.readthedocs.io/en/release-1.3.x/
[docs-1.2.x]: http://py-trees.readthedocs.io/en/release-1.2.x/
[docs-0.7.x]: http://py-trees.readthedocs.io/en/release-0.7.x/
[docs-0.6.x]: http://py-trees.readthedocs.io/en/release-0.6.x/
[docs-0.5.x]: http://docs.ros.org/kinetic/api/py_trees/html/

[docs-devel-image]: http://img.shields.io/badge/docs-devel-brightgreen.svg?style=plastic
[docs-2.2.x-image]: http://img.shields.io/badge/docs-2.2.x-brightgreen.svg?style=plastic
[docs-2.1.x-image]: http://img.shields.io/badge/docs-2.1.x-brightgreen.svg?style=plastic
[docs-2.0.x-image]: http://img.shields.io/badge/docs-2.0.x-brightgreen.svg?style=plastic
[docs-1.3.x-image]: http://img.shields.io/badge/docs-1.3.x-brightgreen.svg?style=plastic
[docs-1.2.x-image]: http://img.shields.io/badge/docs-1.2.x-brightgreen.svg?style=plastic
[docs-0.7.x-image]: http://img.shields.io/badge/docs-0.7.x-brightgreen.svg?style=plastic
[docs-0.6.x-image]: http://img.shields.io/badge/docs-0.6.x-brightgreen.svg?style=plastic
[docs-0.5.x-image]: http://img.shields.io/badge/docs-0.5.x-brightgreen.svg?style=plastic

[rtd-devel-image]: https://readthedocs.org/projects/py-trees/badge/?version=devel&style=plastic
[rtd-2.2.x-image]: https://readthedocs.org/projects/py-trees/badge/?version=release-2.2.x&style=plastic
[rtd-2.1.x-image]: https://readthedocs.org/projects/py-trees/badge/?version=release-2.1.x&style=plastic
[rtd-2.0.x-image]: https://readthedocs.org/projects/py-trees/badge/?version=release-2.0.x&style=plastic
[rtd-1.3.x-image]: https://readthedocs.org/projects/py-trees/badge/?version=release-1.3.x&style=plastic
[rtd-1.2.x-image]: https://readthedocs.org/projects/py-trees/badge/?version=release-1.2.x&style=plastic
[rtd-0.7.x-image]: https://readthedocs.org/projects/py-trees/badge/?version=release-0.7.x&style=plastic
[rtd-0.6.x-image]: https://readthedocs.org/projects/py-trees/badge/?version=release-0.6.x&style=plastic
[rtd-0.5.x-image]: https://readthedocs.org/projects/py-trees/badge/?version=release-0.5.x&style=plastic
[not-available-docs-image]: http://img.shields.io/badge/docs-n/a-yellow.svg?style=plastic

[sources-devel]: https://github.com/splintered-reality/py_trees/tree/devel
[sources-2.2.x]: https://github.com/splintered-reality/py_trees/tree/release/2.2.x
[sources-2.1.x]: https://github.com/splintered-reality/py_trees/tree/release/2.1.x
[sources-2.0.x]: https://github.com/splintered-reality/py_trees/tree/release/2.0.x
[sources-1.3.x]: https://github.com/splintered-reality/py_trees/tree/release/1.3.x
[sources-1.2.x]: https://github.com/splintered-reality/py_trees/tree/release/1.2.x
[sources-0.7.x]: https://github.com/splintered-reality/py_trees/tree/release/0.7.x
[sources-0.6.x]: https://github.com/splintered-reality/py_trees/tree/release/0.6.x
[sources-0.5.x]: https://github.com/splintered-reality/py_trees/tree/release/0.5.x

[sources-devel-image]: http://img.shields.io/badge/sources-devel-blue.svg?style=plastic
[sources-2.2.x-image]: http://img.shields.io/badge/sources-2.2.x-blue.svg?style=plastic
[sources-2.1.x-image]: http://img.shields.io/badge/sources-2.1.x-blue.svg?style=plastic
[sources-2.0.x-image]: http://img.shields.io/badge/sources-2.0.x-blue.svg?style=plastic
[sources-1.3.x-image]: http://img.shields.io/badge/sources-1.3.x-blue.svg?style=plastic
[sources-1.2.x-image]: http://img.shields.io/badge/sources-1.2.x-blue.svg?style=plastic
[sources-0.7.x-image]: http://img.shields.io/badge/sources-0.7.x-blue.svg?style=plastic
[sources-0.6.x-image]: http://img.shields.io/badge/sources-0.6.x-blue.svg?style=plastic
[sources-0.5.x-image]: http://img.shields.io/badge/sources-0.5.x-blue.svg?style=plastic
