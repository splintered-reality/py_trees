# Py Trees

This is a python implementation of behaviour trees designed to facilitate the rapid development
of medium sized decision making engines for use in fields like robotics. Brief feature list:

* Sequence, Selector, Parallel and Chooser composites
* Blackboards for data sharing
* Python generators for smarter ticking over the tree graph
* Python decorators for enabling meta behaviours
* Render trees to dot graphs or visualise with ascii graphs on stdout

Detailed api reference and demo instructions can be found in the [sphinx documentation](http://py-trees.readthedocs.io/) for the package. There is also the [py_trees_ros](https://github.com/stonier/py_trees_ros/tree/devel) package which includes additional modules and documentation for using py_trees with ROS.

## Sources, Builds & Docs

| Devel | Melodic | Kinetic |
|:---:|:---:|:---:|
| [![devel-Sources][devel-sources-image]][devel-sources] | [![0.6.x-Sources][0.6.x-sources-image]][0.6.x-sources] | [![0.5.x-Sources][0.5.x-sources-image]][0.5.x-sources] |
| [![devel-Status][devel-build-status-image]][devel-build-status] | [![melodic-Status][melodic-build-status-image]][melodic-build-status] | [![kinetic-Status][kinetic-build-status-image]][kinetic-build-status] | |
| [![devel-Docs][devel-docs-image]][devel-docs] | [![0.6.x-Docs][0.5.x-docs-image]][0.6.x-docs] | [![0.5.x-Docs][0.5.x-docs-image]][0.5.x-docs] | |

[license-image]: https://img.shields.io/badge/License-BSD%203--Clause-orange.svg?style=plastic
[license]: LICENSE

[devel-sources-image]: http://img.shields.io/badge/sources-devel-blue.svg?style=plastic
[devel-sources]: https://github.com/stonier/py_trees/tree/devel
[0.6.x-sources-image]: http://img.shields.io/badge/sources-0.6.x--melodic-blue.svg?style=plastic
[0.6.x-sources]: https://github.com/stonier/py_trees/tree/release/0.6-melodic
[0.5.x-sources-image]: http://img.shields.io/badge/sources-0.5.x--kinetic-blue.svg?style=plastic
[0.5.x-sources]: https://github.com/stonier/py_trees/tree/release/0.5-kinetic
[0.4.x-sources-image]: http://img.shields.io/badge/sources-0.4.x--indigo--kinetic-blue.svg?style=plastic
[0.4.x-sources]: https://github.com/stonier/py_trees/tree/release/0.4-indigo-kinetic

[devel-build-status-image]: http://build.ros.org/job/Mdev__py_trees__ubuntu_bionic_amd64/badge/icon?style=plastic
[devel-build-status]: http://build.ros.org/job/Mdev__py_trees__ubuntu_bionic_amd64
[melodic-build-status-image]: http://build.ros.org/job/Mbin_uB64__py_trees__ubuntu_bionic_amd64__binary/badge/icon?style=plastic
[melodic-build-status]: http://build.ros.org/job/Mbin_uX64__py_trees__ubuntu_bionic_amd64__binary
[kinetic-build-status-image]: http://build.ros.org/job/Kbin_uX64__py_trees__ubuntu_xenial_amd64__binary/badge/icon?style=plastic
[kinetic-build-status]: http://build.ros.org/job/Kbin_uX64__py_trees__ubuntu_xenial_amd64__binary

[devel-docs-image]: https://readthedocs.org/projects/py-trees/badge/?version=devel&style=plastic
[devel-docs]: http://py-trees.readthedocs.io/
[0.6.x-docs-image]: https://readthedocs.org/projects/py-trees/badge/?version=release-0.6-melodic&style=plastic
[0.6.x-docs]: http://py-trees.readthedocs.io/en/release-0.6-melodic/
[0.5.x-docs-image]: https://readthedocs.org/projects/py-trees/badge/?version=release-0.5-kinetic&style=plastic
[0.5.x-docs]: http://py-trees.readthedocs.io/en/release-0.5-kinetic/


## Getting Started

**Installation**

From [ppa](https://launchpad.net/~d-stonier/+archive/ubuntu/snorriheim) on Ubuntu/Xenial

```
sudo apt install python-py-trees
```

From [pypi](https://pypi.python.org/pypi/py_trees):

```
pip install py_trees
```

Or in a sandboxed ROS Kinetic environment (coming soon):

```
sudo apt install ros-kinetic-py-trees
```

**Development**

You can develop in either a virtualenv (python style):

```
# python 2
source ./virtualenv.bash
# python 3
source ./virtualenv3.bash
```

or in a catkin environment alongside other ROS py-trees packages:

* https://github.com/stonier/repos_index/blob/devel/kinetic/py_trees.repos

