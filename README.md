# Py Trees

[[About](#about)][[Documentation](#documentation)][[Getting Started](#getting-started)][[Releases](#releases)][[Installation](#installation)][[PyTrees-Ros Ecosystem](#pytrees-ros-ecosystem)][[Developers](#developers)]

----

## About

PyTrees is a python implementation of behaviour trees designed to facilitate the rapid development of medium sized decision making engines for use in fields like robotics. Brief feature list:

* Sequence, Selector, Parallel composites
* Blackboards for data sharing
* Python generators for smarter ticking over the tree graph
* Python decorators for enabling meta behaviours
* Render trees to dot graphs or visualise with ascii graphs on stdout

## Documentation

[![devel][docs-devel-image]][docs-devel] [![2.1.x][docs-2.1.x-image]][docs-2.1.x] [![0.7.x][docs-0.7.x-image]][docs-0.7.x] [![0.6.x][docs-0.6.x-image]][docs-0.6.x]

## Getting Started

VSCode and a [py38 DevContainer](.devcontainer/devcontainer.json) on CodeSpaces will set you up with an environment
for running demos, tests and/or creating PRs in less than five minutes. If you're just looking for releases,
fetch them from  [PyPi](https://pypi.org/project/py-trees/).

1. Fork the project to your personal account
2. Click on Code -> Codespaces -> + Create a Codespace
3. Be froody

```
(docker) zen@py_trees:/workspaces/py_trees$ poetry install
(docker) zen@py_trees:/workspaces/py_trees$ poetry shell

(py-trees-py3.8) (docker) zen@py_trees:/workspaces/py_trees$ py-trees-demo-<tab>-<tab>
py-trees-demo-action-behaviour            py-trees-demo-context-switching           py-trees-demo-logging
py-trees-demo-behaviour-lifecycle         py-trees-demo-display-modes               py-trees-demo-pick-up-where-you-left-off
py-trees-demo-blackboard                  py-trees-demo-dot-graphs                  py-trees-demo-selector
py-trees-demo-blackboard-namespaces       py-trees-demo-either-or                   py-trees-demo-sequence
py-trees-demo-blackboard-remappings       py-trees-demo-eternal-guard               py-trees-demo-tree-stewardship

(py-trees-py3.8) (docker) zen@py_trees:/workspaces/py_trees$ py-trees-demo-blackboard
*******************************************************************************
                                   Blackboard
*******************************************************************************

Demonstrates usage of the blackboard and related behaviours.

A sequence is populated with a few behaviours that exercise
reading and writing on the Blackboard in interesting ways.

*******************************************************************************

[DEBUG] Writer               : BlackboardWriter.__init__()
...
...
```

If you're really looking for something more edifying than hello world examples, walk through the [ros tutorials](https://py-trees-ros-tutorials.readthedocs.io/en/release-2.0.x/index.html) which incrementally step through the process of building a scenario handling layer for a robot.

There are also runtime visualisation tools - refer to the [py_trees_ros_viewer/README](https://github.com/splintered-reality/py_trees_ros_viewer/blob/devel/README.md) as an example implementation of the underlying [py_trees_js](https://github.com/splintered-reality/py_trees_js) library. 

## Releases

* `0.y.x` - first open source releases
* `1.0.x` - first stable release
* `1.1.x` - improvements
* `1.2.x` - improvements
* `2.0.x` - blackboards v2 with namespaces, access permissions and key tracking
* `2.1.x` - Chooser deprecated, api housekeeping

| | Devel | 2.1.x | 2.0.x | 1.2.x | 0.7.x | 0.6.x |
|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
| Sources | [![devel][sources-devel-image]][sources-devel] | [![2.1.x][sources-2.1.x-image]][sources-2.1.x] | [![2.0.x][sources-2.0.x-image]][sources-2.0.x] | [![1.2.x][sources-1.2.x-image]][sources-1.2.x] | [![0.7.x][sources-0.7.x-image]][sources-0.7.x] | [![0.6.x][sources-0.6.x-image]][sources-0.6.x]
| Compatibility | [![Python 3.6][python36-image]][python36-docs] | [![Python 3.6][python36-image]][python36-docs] | [![Python 3.6][python36-image]][python36-docs] | [![Python 3.6][python36-image]][python36-docs] | [![Python 3.6][python36-image]][python36-docs] | [![Python 2.7][python27-image]][python27-docs]
| CI | [![devel-Status][devel-build-status-image]][devel-build-status] | [![2.1.x-Status][2.1.x-build-status-image]][2.1.x-build-status] | [![2.0.x-Status][2.0.x-build-status-image]][2.0.x-build-status] | [![1.2.x-Status][1.2.x-build-status-image]][1.2.x-build-status] | [![0.7.x-Status][0.7.x-build-status-image]][0.7.x-build-status] | [![0.6.x-Status][0.6.x-build-status-image]][0.6.x-build-status] 
| Documentation | [![devel-Docs][rtd-devel-image]][docs-devel] | [![2.1.x-Docs][rtd-2.1.x-image]][docs-2.1.x] | [![2.0.x-Docs][rtd-2.0.x-image]][docs-2.0.x] | [![1.2.x-Docs][rtd-1.2.x-image]][docs-1.2.x] | [![0.7.x-Docs][rtd-0.7.x-image]][docs-0.7.x] | [![0.6.x-Docs][rtd-0.6.x-image]][docs-0.6.x]

## Installation

From [ppa](https://launchpad.net/~d-stonier/+archive/ubuntu/snorriheim) on Ubuntu/Bionic:

```
$ sudo apt install python3-py-trees
```

From [pypi](https://pypi.python.org/pypi/py_trees):

```
$ pip3 install py_trees
```

In a Python Virtual Environment:

```
$ git clone https://github.com/splintered-reality/py_trees
$ cd py_trees
$ source ./venv.bash
```

Build your own python3 deb:

```
$ git clone https://github.com/splintered-reality/py_trees
$ cd py_trees
$ source ./venv.bash
$ make deb
```

From the ROS2 ecosystem:

```
$ sudo apt install ros-<rosdistro>-py-trees
```

## PyTrees-ROS Ecosystem

See the `py_trees_ros` [README](https://github.com/splintered-reality/py_trees_ros/blob/devel/README.md) for the latest information on pytrees packages in the ROS ecosystem and their status.

## Developers

### Documentation

```
# Build Locally
$ make -C docs html

# ReadTheDocs requirements
$ poetry export -f requirements.txt --with docs -o docs/requirements.txt
```

[license-image]: https://img.shields.io/badge/License-BSD%203--Clause-orange.svg?style=plastic
[license]: LICENSE

[python36-image]: https://img.shields.io/badge/python-3.6-green.svg?style=plastic
[python36-docs]: https://docs.python.org/3.6/
[python27-image]: https://img.shields.io/badge/python-2.7-green.svg?style=plastic
[python27-docs]: https://docs.python.org/2.7/

[devel-build-status-image]: https://img.shields.io/circleci/build/github/splintered-reality/py_trees?branch=devel&style=plastic
[devel-build-status]: https://circleci.com/gh/splintered-reality/py_trees/tree/devel
[2.1.x-build-status-image]: https://img.shields.io/circleci/build/github/splintered-reality/py_trees?branch=release%2F2.1.x&style=plastic
[2.1.x-build-status]: https://circleci.com/gh/splintered-reality/py_trees/tree/release/2.1.x
[2.0.x-build-status-image]: https://img.shields.io/circleci/build/github/splintered-reality/py_trees?branch=release%2F2.0.x&style=plastic
[2.0.x-build-status]: https://circleci.com/gh/splintered-reality/py_trees/tree/release/2.0.x
[1.3.x-build-status-image]: https://img.shields.io/circleci/build/github/splintered-reality/py_trees?branch=release%2F1.3.x&style=plastic
[1.3.x-build-status]: https://circleci.com/gh/splintered-reality/py_trees/tree/release/1.3.x
[1.2.x-build-status-image]: https://img.shields.io/circleci/build/github/splintered-reality/py_trees?branch=release%2F1.2.x&style=plastic
[1.2.x-build-status]: https://circleci.com/gh/splintered-reality/py_trees/tree/release/1.2.x
[0.7.x-build-status-image]: https://img.shields.io/circleci/build/github/splintered-reality/py_trees?branch=release%2F0.7.x&style=plastic
[0.7.x-build-status]: https://circleci.com/gh/splintered-reality/py_trees/tree/release/0.7.x
[0.6.x-build-status-image]: https://img.shields.io/circleci/build/github/splintered-reality/py_trees?branch=release%2F0.6x&style=plastic
[0.6.x-build-status]: https://circleci.com/gh/splintered-reality/py_trees/tree/release/0.6.x

[docs-devel]: http://py-trees.readthedocs.io/
[docs-2.1.x]: http://py-trees.readthedocs.io/en/release-2.1.x/
[docs-2.0.x]: http://py-trees.readthedocs.io/en/release-2.0.x/
[docs-1.3.x]: http://py-trees.readthedocs.io/en/release-1.3.x/
[docs-1.2.x]: http://py-trees.readthedocs.io/en/release-1.2.x/
[docs-0.7.x]: http://py-trees.readthedocs.io/en/release-0.7.x/
[docs-0.6.x]: http://py-trees.readthedocs.io/en/release-0.6.x/
[docs-0.5.x]: http://docs.ros.org/kinetic/api/py_trees/html/

[docs-devel-image]: http://img.shields.io/badge/docs-devel-brightgreen.svg?style=plastic
[docs-2.1.x-image]: http://img.shields.io/badge/docs-2.1.x-brightgreen.svg?style=plastic
[docs-2.0.x-image]: http://img.shields.io/badge/docs-2.0.x-brightgreen.svg?style=plastic
[docs-1.3.x-image]: http://img.shields.io/badge/docs-1.3.x-brightgreen.svg?style=plastic
[docs-1.2.x-image]: http://img.shields.io/badge/docs-1.2.x-brightgreen.svg?style=plastic
[docs-0.7.x-image]: http://img.shields.io/badge/docs-0.7.x-brightgreen.svg?style=plastic
[docs-0.6.x-image]: http://img.shields.io/badge/docs-0.6.x-brightgreen.svg?style=plastic
[docs-0.5.x-image]: http://img.shields.io/badge/docs-0.5.x-brightgreen.svg?style=plastic

[rtd-devel-image]: https://readthedocs.org/projects/py-trees/badge/?version=devel&style=plastic
[rtd-2.1.x-image]: https://readthedocs.org/projects/py-trees/badge/?version=release-2.1.x&style=plastic
[rtd-2.0.x-image]: https://readthedocs.org/projects/py-trees/badge/?version=release-2.0.x&style=plastic
[rtd-1.3.x-image]: https://readthedocs.org/projects/py-trees/badge/?version=release-1.3.x&style=plastic
[rtd-1.2.x-image]: https://readthedocs.org/projects/py-trees/badge/?version=release-1.2.x&style=plastic
[rtd-0.7.x-image]: https://readthedocs.org/projects/py-trees/badge/?version=release-0.7.x&style=plastic
[rtd-0.6.x-image]: https://readthedocs.org/projects/py-trees/badge/?version=release-0.6.x&style=plastic
[rtd-0.5.x-image]: https://readthedocs.org/projects/py-trees/badge/?version=release-0.5.x&style=plastic
[not-available-docs-image]: http://img.shields.io/badge/docs-n/a-yellow.svg?style=plastic

[sources-devel]: https://github.com/splintered-reality/py_trees/tree/devel
[sources-2.1.x]: https://github.com/splintered-reality/py_trees/tree/release/2.1.x
[sources-2.0.x]: https://github.com/splintered-reality/py_trees/tree/release/2.0.x
[sources-1.3.x]: https://github.com/splintered-reality/py_trees/tree/release/1.3.x
[sources-1.2.x]: https://github.com/splintered-reality/py_trees/tree/release/1.2.x
[sources-0.7.x]: https://github.com/splintered-reality/py_trees/tree/release/0.7.x
[sources-0.6.x]: https://github.com/splintered-reality/py_trees/tree/release/0.6.x
[sources-0.5.x]: https://github.com/splintered-reality/py_trees/tree/release/0.5.x

[sources-devel-image]: http://img.shields.io/badge/sources-devel-blue.svg?style=plastic
[sources-2.1.x-image]: http://img.shields.io/badge/sources-2.1.x-blue.svg?style=plastic
[sources-2.0.x-image]: http://img.shields.io/badge/sources-2.0.x-blue.svg?style=plastic
[sources-1.3.x-image]: http://img.shields.io/badge/sources-1.3.x-blue.svg?style=plastic
[sources-1.2.x-image]: http://img.shields.io/badge/sources-1.2.x-blue.svg?style=plastic
[sources-0.7.x-image]: http://img.shields.io/badge/sources-0.7.x-blue.svg?style=plastic
[sources-0.6.x-image]: http://img.shields.io/badge/sources-0.6.x-blue.svg?style=plastic
[sources-0.5.x-image]: http://img.shields.io/badge/sources-0.5.x-blue.svg?style=plastic
