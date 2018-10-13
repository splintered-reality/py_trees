# Py Trees

## About

This is a python implementation of behaviour trees designed to facilitate the rapid development
of medium sized decision making engines for use in fields like robotics. Brief feature list:

* Sequence, Selector, Parallel and Chooser composites
* Blackboards for data sharing
* Python generators for smarter ticking over the tree graph
* Python decorators for enabling meta behaviours
* Render trees to dot graphs or visualise with ascii graphs on stdout

**Sphinx Documentation**

Detailed api reference and demo instructions can be found on read-the-docs
([devel](http://py-trees.readthedocs.io/), [0.7.x](http://py-trees.readthedocs.io/en/release-0.7.x/), [0.6.x](http://py-trees.readthedocs.io/en/release-0.6.x/)).

**ROS**

This package is a pure python-only package, however you can find additional modules and documentation for
using py_trees with ROS 1 in the [py_trees_ros](https://github.com/stonier/py_trees_ros/tree/devel) package.

**Python2 vs Python3**

Release 0.6.x supported both Python2 and Python3. 0.7.x and onwards will only have official support (bugs and doc.) for python3.

## Sources, Builds & Docs

| Devel | 0.7.x/Bouncy | 0.6.x/Melodic | 0.5.x/Kinetic |
|:---:|:---:|:---:|:---:|
| [![devel-Sources][devel-sources-image]][devel-sources] | [![0.7.x-Sources][0.7.x-sources-image]][0.7.x-sources] | [![0.6.x-Sources][0.6.x-sources-image]][0.6.x-sources] | [![0.5.x-Sources][0.5.x-sources-image]][0.5.x-sources] |
| [![devel-Status][devel-build-status-image]][devel-build-status] | [![0.7.x-Status][0.7.x-build-status-image]][0.7.x-build-status] | [![melodic-Status][melodic-build-status-image]][melodic-build-status] | [![kinetic-Status][kinetic-build-status-image]][kinetic-build-status] | |
| [![devel-Docs][devel-docs-image]][devel-docs] | [![0.7.x-Docs][0.7.x-docs-image]][0.7.x-docs] | [![0.6.x-Docs][0.6.x-docs-image]][0.6.x-docs] | [![0.5.x-Docs][0.5.x-docs-image]][0.5.x-docs] | |

[license-image]: https://img.shields.io/badge/License-BSD%203--Clause-orange.svg?style=plastic
[license]: LICENSE

[devel-sources-image]: http://img.shields.io/badge/sources-devel-blue.svg?style=plastic
[devel-sources]: https://github.com/stonier/py_trees/tree/devel
[0.7.x-sources-image]: http://img.shields.io/badge/sources-0.7.x-blue.svg?style=plastic
[0.7.x-sources]: https://github.com/stonier/py_trees/tree/release/0.7.x
[0.6.x-sources-image]: http://img.shields.io/badge/sources-0.6.x-blue.svg?style=plastic
[0.6.x-sources]: https://github.com/stonier/py_trees/tree/release/0.6.x
[0.5.x-sources-image]: http://img.shields.io/badge/sources-0.5.x-blue.svg?style=plastic
[0.5.x-sources]: https://github.com/stonier/py_trees/tree/release/0.5.x
[0.4.x-sources-image]: http://img.shields.io/badge/sources-0.4.x--indigo--kinetic-blue.svg?style=plastic
[0.4.x-sources]: https://github.com/stonier/py_trees/tree/release/0.4-indigo-kinetic

[devel-build-status-image]: http://build.ros.org/job/Mbin_uB64__py_trees__ubuntu_bionic_amd64__binary/badge/icon?style=plastic
[devel-build-status]: https://circleci.com/gh/stonier/py_trees/tree/devel
[0.7.x-build-status-image]: http://build.ros.org/job/Mbin_uB64__py_trees__ubuntu_bionic_amd64__binary/badge/icon?style=plastic
[0.7.x-build-status]: https://circleci.com/gh/stonier/py_trees/tree/release/0.7.x
[bouncy-build-status-image]: http://build.ros.org/job/Mbin_uB64__py_trees__ubuntu_bionic_amd64__binary/badge/icon?style=plastic
[bouncy-build-status]: http://build.ros.org/job/Mbin_uX64__py_trees__ubuntu_bionic_amd64__binary
[melodic-build-status-image]: http://build.ros.org/job/Mbin_uB64__py_trees__ubuntu_bionic_amd64__binary/badge/icon?style=plastic
[melodic-build-status]: http://build.ros.org/job/Mbin_uX64__py_trees__ubuntu_bionic_amd64__binary
[kinetic-build-status-image]: http://build.ros.org/job/Kbin_uX64__py_trees__ubuntu_xenial_amd64__binary/badge/icon?style=plastic
[kinetic-build-status]: http://build.ros.org/job/Kbin_uX64__py_trees__ubuntu_xenial_amd64__binary

[devel-docs-image]: https://readthedocs.org/projects/py-trees/badge/?version=devel&style=plastic
[devel-docs]: http://py-trees.readthedocs.io/
[0.7.x-docs-image]: https://readthedocs.org/projects/py-trees/badge/?version=release-0.7.x&style=plastic
[0.7.x-docs]: http://py-trees.readthedocs.io/en/release-0.7.x/
[0.6.x-docs-image]: https://readthedocs.org/projects/py-trees/badge/?version=release-0.6.x&style=plastic
[0.6.x-docs]: http://py-trees.readthedocs.io/en/release-0.6.x/
[0.5.x-docs-image]: https://readthedocs.org/projects/py-trees/badge/?version=release-0.5.x&style=plastic
[0.5.x-docs]: http://py-trees.readthedocs.io/en/release-0.5.x/


## Getting Started

**Installation**


From [ppa](https://launchpad.net/~d-stonier/+archive/ubuntu/snorriheim) on Ubuntu/Bionic

```
sudo apt install python3-py-trees
```

From [pypi](https://pypi.python.org/pypi/py_trees):

```
$ pip install py_trees
```

Or in a ROS2 environment:

```
$ sudo apt install ros-<rosdistro>-py-trees
```

Build your own python3 deb:

```
$ source ./virtualenv.bash
$ make deb
```


**Development Environment**

You can develop in either a virtualenv (python3):

```
$ source ./virtualenv.bash
```

or in a ament-colcon environment:

```
$ mkdir src
$ git clone https://github.com/stonier/py_trees src/py_trees
$ colcon build
$ colcon test
```

**Demos and Tutorials**

Move on to the documentation ([devel](http://py-trees.readthedocs.io/), [0.7.x](http://py-trees.readthedocs.io/en/release-0.7.x/), [0.6.x](http://py-trees.readthedocs.io/en/release-0.6.x/))!