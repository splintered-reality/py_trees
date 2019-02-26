# Py Trees

## Status

| Devel | 0.8.x | 0.6.x | 0.5.x |
|:---:|:---:|:---:|:---:|
| [![devel-Sources][devel-sources-image]][devel-sources] | [![0.8.x-Sources][0.8.x-sources-image]][0.8.x-sources] | [![0.6.x-Sources][0.6.x-sources-image]][0.6.x-sources] | [![0.5.x-Sources][0.5.x-sources-image]][0.5.x-sources] |
| [![devel-Status][devel-build-status-image]][devel-build-status] | [![0.8.x-Status][0.8.x-build-status-image]][0.8.x-build-status] | [![melodic-Status][melodic-build-status-image]][melodic-build-status] | [![kinetic-Status][kinetic-build-status-image]][kinetic-build-status] | |
| [![devel-Docs][devel-docs-image]][devel-docs] | [![0.8.x-Docs][0.8.x-docs-image]][0.8.x-docs] | [![0.6.x-Docs][0.6.x-docs-image]][0.6.x-docs] | [![0.5.x-Docs][0.5.x-docs-image]][0.5.x-docs] | |

[license-image]: https://img.shields.io/badge/License-BSD%203--Clause-orange.svg?style=plastic
[license]: LICENSE

[devel-sources-image]: http://img.shields.io/badge/sources-devel-blue.svg?style=plastic
[devel-sources]: https://github.com/splintered-reality/py_trees/tree/devel
[0.8.x-sources]: https://github.com/splintered-reality/py_trees/tree/release/0.8.x
[0.7.x-sources]: https://github.com/splintered-reality/py_trees/tree/release/0.7.x
[0.6.x-sources]: https://github.com/splintered-reality/py_trees/tree/release/0.6.x
[0.5.x-sources]: https://github.com/splintered-reality/py_trees/tree/release/0.5.x
[0.4.x-sources]: https://github.com/splintered-reality/py_trees/tree/release/0.4-indigo-kinetic

[devel-build-status-image]: http://build.ros.org/job/Mbin_uB64__py_trees__ubuntu_bionic_amd64__binary/badge/icon?style=plastic
[devel-build-status]: https://circleci.com/gh/splintered-reality/py_trees/tree/devel
[0.8.x-build-status-image]: http://build.ros.org/job/Mbin_uB64__py_trees__ubuntu_bionic_amd64__binary/badge/icon?style=plastic
[0.8.x-build-status]: https://circleci.com/gh/splintered-reality/py_trees/tree/release/0.8.x
[0.7.x-build-status-image]: http://build.ros.org/job/Mbin_uB64__py_trees__ubuntu_bionic_amd64__binary/badge/icon?style=plastic
[0.7.x-build-status]: https://circleci.com/gh/splintered-reality/py_trees/tree/release/0.7.x
[bouncy-build-status-image]: http://build.ros.org/job/Mbin_uB64__py_trees__ubuntu_bionic_amd64__binary/badge/icon?style=plastic
[bouncy-build-status]: http://build.ros.org/job/Mbin_uX64__py_trees__ubuntu_bionic_amd64__binary
[melodic-build-status-image]: http://build.ros.org/job/Mbin_uB64__py_trees__ubuntu_bionic_amd64__binary/badge/icon?style=plastic
[melodic-build-status]: http://build.ros.org/job/Mbin_uX64__py_trees__ubuntu_bionic_amd64__binary
[kinetic-build-status-image]: http://build.ros.org/job/Kbin_uX64__py_trees__ubuntu_xenial_amd64__binary/badge/icon?style=plastic
[kinetic-build-status]: http://build.ros.org/job/Kbin_uX64__py_trees__ubuntu_xenial_amd64__binary

[devel-docs-image]: https://readthedocs.org/projects/py-trees/badge/?version=devel&style=plastic
[devel-docs]: http://py-trees.readthedocs.io/
[0.8.x-docs-image]: https://readthedocs.org/projects/py-trees/badge/?version=release-0.8.x&style=plastic
[0.8.x-docs]: http://py-trees.readthedocs.io/en/release-0.8.x/
[0.7.x-docs-image]: https://readthedocs.org/projects/py-trees/badge/?version=release-0.7.x&style=plastic
[0.7.x-docs]: http://py-trees.readthedocs.io/en/release-0.7.x/
[0.6.x-docs-image]: https://readthedocs.org/projects/py-trees/badge/?version=release-0.6.x&style=plastic
[0.6.x-docs]: http://py-trees.readthedocs.io/en/release-0.6.x/
[0.5.x-docs-image]: https://readthedocs.org/projects/py-trees/badge/?version=release-0.5.x&style=plastic
[0.5.x-docs]: http://py-trees.readthedocs.io/en/release-0.5.x/

## About

This is a python3 implementation of behaviour trees designed to facilitate the rapid development of medium sized decision making engines for use in fields like robotics. Brief feature list:

* Sequence, Selector, Parallel and Chooser composites
* Blackboards for data sharing
* Python generators for smarter ticking over the tree graph
* Python decorators for enabling meta behaviours
* Render trees to dot graphs or visualise with ascii graphs on stdout

Note: Official python2 support was dropped in the 0.6.x releases although it may 'just work'.

## Installation

From [ppa](https://launchpad.net/~d-stonier/+archive/ubuntu/snorriheim) on Ubuntu/Bionic:

```
$ sudo apt install python3-py-trees
```

From [pypi](https://pypi.python.org/pypi/py_trees):

```
$ pip install py_trees
```

From the ROS2 ecosystem:

```
$ sudo apt install ros-<rosdistro>-py-trees
```

In a Python Virtual Environment:

```
$ git clone https://github.com/splintered-reality/py_trees
$ cd py_trees
$ source ./virtualenv.bash
```

Build your own python3 deb:

```
$ git clone https://github.com/splintered-reality/py_trees
$ cd py_trees
$ source ./virtualenv.bash
$ make deb
```

## Demos and Tutorials

API, guides and demo instructions can be found in the sphinx generated documetnation on Read-The-Docs ([devel](http://py-trees.readthedocs.io/), [0.8.x](http://py-trees.readthedocs.io/en/release-0.8.x/), [0.6.x](http://py-trees.readthedocs.io/en/release-0.6.x/))!

## PyTrees in ROS

This repository is python-only, however additional modules & documentation are available for using `py_trees` with `ROS` that provide ROS-specific behaviours, logging, tools and visualisations in ROS.

| | [ROS2/Bouncy][bouncy-repository] | [ROS1/Melodic][melodic-repository] | [ROS1/Kinetic][kinetic-repository] |
|:---:|:---:|:---:|:---:|
| py_trees | [![0.8.x][0.8.x-sources-image]][py-trees-0.8.x] | [![0.6.x][0.6.x-sources-image]][py-trees-0.6.x] | [![0.5.x][0.5.x-sources-image]][py-trees-0.5.x] |
| py_trees-msgs | - | [![0.3.x][0.3.x-sources-image]][py-trees-msgs-melodic] | [![0.3.x][0.3.x-sources-image]][py-trees-msgs-kinetic] |
| py_trees-ros | - | [![0.5.x][0.5.x-sources-image]][py-trees-ros-melodic] | [![0.5.x][0.5.x-sources-image]][py-trees-ros-kinetic] |
| rqt_py_trees | - | [![0.3.x][0.3.x-sources-image]][rqt-py-trees-melodic] | [![0.3.x][0.3.x-sources-image]][rqt-py-trees-kinetic] |

[0.8.x-sources-image]: http://img.shields.io/badge/sources-0.8.x-blue.svg?style=plastic
[0.7.x-sources-image]: http://img.shields.io/badge/sources-0.7.x-blue.svg?style=plastic
[0.6.x-sources-image]: http://img.shields.io/badge/sources-0.6.x-blue.svg?style=plastic
[0.5.x-sources-image]: http://img.shields.io/badge/sources-0.5.x-blue.svg?style=plastic
[0.4.x-sources-image]: http://img.shields.io/badge/sources-0.4.x-blue.svg?style=plastic
[0.3.x-sources-image]: http://img.shields.io/badge/sources-0.3.x-blue.svg?style=plastic

[bouncy-repository]: http://repo.ros2.org/status_page/ros_bouncy_default.html?q=py_trees
[melodic-repository]: http://repositories.ros.org/status_page/ros_melodic_default.html?q=py_trees
[kinetic-repository]: http://repositories.ros.org/status_page/ros_kinetic_default.html?q=py_trees

[py-trees-0.8.x]: https://github.com/splintered-reality/py_trees/tree/release/0.8.x
[py-trees-0.7.x]: https://github.com/splintered-reality/py_trees/tree/release/0.7.x
[py-trees-0.6.x]: https://github.com/splintered-reality/py_trees/tree/release/0.6.x
[py-trees-0.5.x]: https://github.com/splintered-reality/py_trees/tree/release/0.5.x
[py-trees-msgs-kinetic]: https://github.com/stonier/py_trees_msgs/tree/release/0.3-kinetic
[py-trees-msgs-melodic]: https://github.com/stonier/py_trees_msgs/tree/release/0.3-melodic
[py-trees-ros-kinetic]: https://github.com/splintered-reality/py_trees_ros/tree/release/0.5-kinetic
[py-trees-ros-melodic]: https://github.com/splintered-reality/py_trees_ros/tree/release/0.5-melodic
[rqt-py-trees-kinetic]: https://github.com/splintered-reality/rqt_py_trees/tree/release/0.3-kinetic
[rqt-py-trees-melodic]: https://github.com/splintered-reality/rqt_py_trees/tree/release/0.3-melodic
