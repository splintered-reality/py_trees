# PyTrees

[[About](#about)] [[What's New?](#whats-new)] [[Documentation](#documentation)] [[Getting Started](#getting-started)] [[Next Steps](#next-steps)] [[Maintainers](#maintainers)] [[Acknowledgments](#acknowledgments)] [[Releases](#releases)]

----

## About

PyTrees is a Python implementation of behaviour trees designed to facilitate the rapid development of medium sized decision making engines for use in fields like robotics.

Brief feature list:

* Behaviours, Decorators, Sequences, Selectors, Parallels and BehaviourTree.
* Blackboards for data sharing.
* A useful library of behaviours, decorators, and idioms.
* A more abstract ports API and XML parser for declarative behavior definitions.
* Serialise to a dot graph or render to ascii/unicode in a terminal.
* Tested on Linux and Mac (YMMV with Windows).

## What's New?

* [2026-06-27] Modernized project infrastructure with `uv`, added support for Python 3.14.
* [2026-05-19] Typed input/output ports for behaviours, with XML parser support.
* [2025-11-13] New `ForEach` decorator.
* [2025-11-13] New `CompareBlackboardVariables` behaviour, with comparison expressions that allow callables.
* [2025-01-11] Support for Python 3.12 was added, and Python 3.8 was dropped.

## Documentation

[![devel][docs-devel-image]][docs-devel] [![2.5.x][docs-2.5.x-image]][docs-2.5.x] [![2.4.x][docs-2.4.x-image]][docs-2.4.x] [![2.3.x][docs-2.3.x-image]][docs-2.3.x] [![2.2.x][docs-2.2.x-image]][docs-2.2.x]

## Getting Started

Released versions of PyTrees can be installed directly from:

* [PyPi](https://pypi.org/project/py_trees/): `pip install py_trees`
* [conda-forge](https://anaconda.org/channels/conda-forge/packages/py-trees/overview): `conda install conda-forge::py-trees`

Alternatively, you can clone the repo, install [`uv`](https://docs.astral.sh/uv/), and you're up and running in under 5 minutes:

```
# Install dependencies into a .venv
$ uv sync

# Explore the demos
$ uv run py-trees-demo-<tab>-<tab>
py-trees-demo-action-behaviour            py-trees-demo-context-switching           py-trees-demo-logging
py-trees-demo-behaviour-lifecycle         py-trees-demo-display-modes               py-trees-demo-pick-up-where-you-left-off
py-trees-demo-blackboard                  py-trees-demo-dot-graphs                  py-trees-demo-selector
py-trees-demo-blackboard-namespaces       py-trees-demo-either-or                   py-trees-demo-sequence
py-trees-demo-blackboard-remappings       py-trees-demo-eternal-guard               py-trees-demo-tree-stewardship
$ uv run py-trees-demo-blackboard
...

# Hack some Code

# Run the Formatter, Linter, Type-Checker and Tests
$ uv run ruff format
$ uv run ruff check
$ uv run ty check
$ uv run pytest -s tests/

# Contribute a PR!
# https://github.com/splintered-reality/py_trees/blob/devel/CONTRIBUTING.md
```

See [DEVELOPING.md](DEVELOPING.md) for more detail.
And be froody.

## Next Steps

On PyPi:
* [py_trees](https://pypi.org/project/py-trees/)
* [py_trees_js](https://pypi.org/project/py-trees-js/)

Examples:
* [ReadTheDocs - PyTrees ROS Tutorials](https://py-trees-ros-tutorials.readthedocs.io/en/devel/index.html) - significantly more edifying than the demos, these incrementally walk through the process of building a decision making layer for a robot. These use ROS 2 (sparsely), but merely browsing should be enlightening regardless.

Visualisation:
* [py_trees_js](https://github.com/splintered-reality/py_trees_js) - a javascript library for building your own runtime visualisation tool

Robotics:
* [py_trees_ros](https://github.com/splintered-reality/py_trees_ros) - a tree manager and behaviours designed for use specifically with `ROS 2`
* [py_trees_ros_viewer](https://github.com/splintered-reality/py_trees_ros_viewer) - a `Qt/ROS 2` implementation of `py_trees_js`

## Maintainers

* Daniel Stonier ([@stonier](https://github.com/stonier))
* Sebastian Castro ([@sea-bass](https://github.com/sea-bass))

## Acknowledgments

Contributors are welcome to add themselves here in future PRs.

* Port declarations, type validation, and XML parser: contributed by [Sunrise Robotics](https://sunriserobotics.co/).

## Releases

* `2.5.x` - New Ports API with XML file parsing, modernized dev environment with `uv`, support Python 3.14.
* `2.4.x` - New ForEach decorator, new CompareBlackboardVariables behaviour, and callables in comparison expressions.
* `2.3.x` - Support for Python 3.12 was added, and Python 3.8 was dropped.
* `2.2.x` - Selectors, Sequences with and without memory. Improved testing and style/type checking.
* `2.1.x` - Chooser deprecated. API housekeeping.
* `2.0.x` - Blackboards V2!
* `1.2.x` - Trees can now shutdown cleanly. StatusToBlackboard and EternalGuard, Visitors get finalise().
* `1.1.x` - Fixes for setup, tick-tock, viz.
* `1.0.x` - Behaviours, Decorators, Composites, Blackboards, Tree Management and Viz tools.
* `0.y.x` - First open source pre-releases.

| | Devel | 2.5.x | 2.4.x | 2.3.x | 2.2.x |
|:---:|:---:|:---:|:---:|:---:|:---:|
| Sources | [![devel][sources-devel-image]][sources-devel] | [![2.5.x][sources-2.5.x-image]][sources-2.5.x] | [![2.4.x][sources-2.4.x-image]][sources-2.4.x] | [![2.3.x][sources-2.3.x-image]][sources-2.3.x] | [![2.2.x][sources-2.2.x-image]][sources-2.2.x] |
| Compatibility | [![Python 3.14][python314-image]][python314-docs]<br/>[![Python 3.12][python312-image]][python312-docs]<br/>[![Python 3.10][python310-image]][python310-docs] | [![Python 3.14][python314-image]][python314-docs]<br/>[![Python 3.12][python312-image]][python312-docs]<br/>[![Python 3.10][python310-image]][python310-docs] | [![Python 3.12][python312-image]][python312-docs]<br/>[![Python 3.10][python310-image]][python310-docs] | [![Python 3.12][python312-image]][python312-docs]<br/>[![Python 3.10][python310-image]][python310-docs] | [![Python 3.10][python310-image]][python310-docs]<br/>[![Python 3.8][python38-image]][python38-docs] |
| CI | [![devel-Status][devel-build-status-image]][devel-build-status] | [![2.5.x-Status][2.5.x-build-status-image]][2.5.x-build-status] | [![2.4.x-Status][2.4.x-build-status-image]][2.4.x-build-status] | [![2.3.x-Status][2.3.x-build-status-image]][2.3.x-build-status] | [![2.2.x-Status][2.2.x-build-status-image]][2.2.x-build-status] |
| Documentation | [![devel-Docs][rtd-devel-image]][docs-devel] | [![2.5.x-Docs][rtd-2.5.x-image]][docs-2.5.x] | [![2.4.x-Docs][rtd-2.4.x-image]][docs-2.4.x] | [![2.3.x-Docs][rtd-2.3.x-image]][docs-2.3.x] | [![2.2.x-Docs][rtd-2.2.x-image]][docs-2.2.x] |


[license-image]: https://img.shields.io/badge/License-BSD%203--Clause-orange.svg?style=plastic
[license]: LICENSE

[python314-image]: https://img.shields.io/badge/python-3.14-green.svg?style=plastic
[python314-docs]: https://docs.python.org/3.14/
[python312-image]: https://img.shields.io/badge/python-3.12-green.svg?style=plastic
[python312-docs]: https://docs.python.org/3.12/
[python310-image]: https://img.shields.io/badge/python-3.10-green.svg?style=plastic
[python310-docs]: https://docs.python.org/3.10/
[python38-image]: https://img.shields.io/badge/python-3.8-green.svg?style=plastic
[python38-docs]: https://docs.python.org/3.8/

[devel-build-status-image]: https://github.com/splintered-reality/py_trees/actions/workflows/pre-merge.yaml/badge.svg
[devel-build-status]: https://github.com/splintered-reality/py_trees/actions/workflows/pre-merge.yaml
[2.5.x-build-status-image]: https://github.com/splintered-reality/py_trees/actions/workflows/pre-merge.yaml/badge.svg?branch=release/2.5.x
[2.5.x-build-status]: https://github.com/splintered-reality/py_trees/actions/workflows/pre-merge.yaml
[2.4.x-build-status-image]: https://github.com/splintered-reality/py_trees/actions/workflows/pre-merge.yaml/badge.svg?branch=release/2.4.x
[2.4.x-build-status]: https://github.com/splintered-reality/py_trees/actions/workflows/pre-merge.yaml
[2.3.x-build-status-image]: https://github.com/splintered-reality/py_trees/actions/workflows/pre-merge.yaml/badge.svg?branch=release/2.3.x
[2.3.x-build-status]: https://github.com/splintered-reality/py_trees/actions/workflows/pre-merge.yaml
[2.2.x-build-status-image]: https://github.com/splintered-reality/py_trees/actions/workflows/pre-merge.yaml/badge.svg?branch=release/2.2.x
[2.2.x-build-status]: https://github.com/splintered-reality/py_trees/actions/workflows/pre-merge.yaml

[docs-devel]: http://py-trees.readthedocs.io/
[docs-2.5.x]: http://py-trees.readthedocs.io/en/release-2.5.x/
[docs-2.4.x]: http://py-trees.readthedocs.io/en/release-2.4.x/
[docs-2.3.x]: http://py-trees.readthedocs.io/en/release-2.3.x/
[docs-2.2.x]: http://py-trees.readthedocs.io/en/release-2.2.x/

[docs-devel-image]: http://img.shields.io/badge/docs-devel-brightgreen.svg?style=plastic
[docs-2.5.x-image]: http://img.shields.io/badge/docs-2.5.x-brightgreen.svg?style=plastic
[docs-2.4.x-image]: http://img.shields.io/badge/docs-2.4.x-brightgreen.svg?style=plastic
[docs-2.3.x-image]: http://img.shields.io/badge/docs-2.3.x-brightgreen.svg?style=plastic
[docs-2.2.x-image]: http://img.shields.io/badge/docs-2.2.x-brightgreen.svg?style=plastic

[rtd-devel-image]: https://readthedocs.org/projects/py-trees/badge/?version=devel&style=plastic
[rtd-2.5.x-image]: https://readthedocs.org/projects/py-trees/badge/?version=release-2.5.x&style=plastic
[rtd-2.4.x-image]: https://readthedocs.org/projects/py-trees/badge/?version=release-2.4.x&style=plastic
[rtd-2.3.x-image]: https://readthedocs.org/projects/py-trees/badge/?version=release-2.3.x&style=plastic
[rtd-2.2.x-image]: https://readthedocs.org/projects/py-trees/badge/?version=release-2.2.x&style=plastic

[sources-devel]: https://github.com/splintered-reality/py_trees/tree/devel
[sources-2.5.x]: https://github.com/splintered-reality/py_trees/tree/release/2.5.x
[sources-2.4.x]: https://github.com/splintered-reality/py_trees/tree/release/2.4.x
[sources-2.3.x]: https://github.com/splintered-reality/py_trees/tree/release/2.3.x
[sources-2.2.x]: https://github.com/splintered-reality/py_trees/tree/release/2.2.x

[sources-devel-image]: http://img.shields.io/badge/sources-devel-blue.svg?style=plastic
[sources-2.5.x-image]: http://img.shields.io/badge/sources-2.5.x-blue.svg?style=plastic
[sources-2.4.x-image]: http://img.shields.io/badge/sources-2.4.x-blue.svg?style=plastic
[sources-2.3.x-image]: http://img.shields.io/badge/sources-2.3.x-blue.svg?style=plastic
[sources-2.2.x-image]: http://img.shields.io/badge/sources-2.2.x-blue.svg?style=plastic
