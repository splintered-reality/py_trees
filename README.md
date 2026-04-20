# PyTrees

[[About](#about)] [[What's New?](#whats-new)] [[Documentation](#documentation)] [[Getting Started](#getting-started)] [[Next Steps](#next-steps)] [[Acknowledgments](#acknowledgments)] [[Releases](#releases)]

----

## About

PyTrees is a Python implementation of behaviour trees designed to facilitate the rapid development of medium sized decision making engines for use in fields like robotics.

Brief feature list:

* Behaviours, Decorators, Sequences, Selectors, Parallels and BehaviourTree.
* Blackboards for data sharing.
* A useful library of behaviours, decorators, and idioms.
* Serialise to a dot graph or render to ascii/unicode in a terminal.
* Tested on Linux and Mac (YMMV with Windows).

## What's New?

* [2025-11-13] New `ForEach` decorator.
* [2025-11-13] New `CompareBlackboardVariables` behaviour, with comparison expressions that allow callables.
* [2025-01-11] Support for Python 3.12 was added, and Python 3.8 was dropped.

## Documentation

[![devel][docs-devel-image]][docs-devel] [![2.4.x][docs-2.4.x-image]][docs-2.4.x] [![2.3.x][docs-2.3.x-image]][docs-2.3.x] [![2.2.x][docs-2.2.x-image]][docs-2.2.x] [![2.1.x][docs-2.1.x-image]][docs-2.1.x]

## Getting Started

You can get started on Codespaces (with no mismatched environment issues and in under 5 minutes) [1]:

1. Fork the project to your personal account
2. Click on Code -> Codespaces -> + Create a Codespace
3. Enter the Terminal

```
# Install Dependencies
(docker) zen@py_trees:/workspaces/py_trees$ poetry install

# Explore the demos
(docker) zen@py_trees:/workspaces/py_trees$ poetry shell
(py-trees-py3.10) (docker) zen@py_trees:/workspaces/py_trees$ py-trees-demo-<tab>-<tab>
py-trees-demo-action-behaviour            py-trees-demo-context-switching           py-trees-demo-logging
py-trees-demo-behaviour-lifecycle         py-trees-demo-display-modes               py-trees-demo-pick-up-where-you-left-off
py-trees-demo-blackboard                  py-trees-demo-dot-graphs                  py-trees-demo-selector
py-trees-demo-blackboard-namespaces       py-trees-demo-either-or                   py-trees-demo-sequence
py-trees-demo-blackboard-remappings       py-trees-demo-eternal-guard               py-trees-demo-tree-stewardship
(py-trees-py3.10) (docker) zen@py_trees:/workspaces/py_trees$ py-trees-demo-blackboard
...
(py-trees-py3.10) (docker) zen@py_trees:/workspaces/py_trees$ exit

# Hack some Code

# Run the Formatter, Tests, Linters and Mypy
(docker) zen@py_trees:/workspaces/py_trees$ poetry run tox -l
py310 py312 format check mypy310 mypy312
(docker) zen@py_trees:/workspaces/py_trees$ poetry run tox -e format
...
(docker) zen@py_trees:/workspaces/py_trees$ poetry run tox -e py310
...
(docker) zen@py_trees:/workspaces/py_trees$ poetry run tox -e check
...

# Contribute a PR!
# https://github.com/splintered-reality/py_trees/blob/devel/CONTRIBUTING.md
```

[1] All of the above will, of course, work in a local environment if you have `poetry` installed.
If you're using `VSCode` you don't even need that, just reopen the project in the [devcontainer](.devcontainer/devcontainer.json) and be froody.

## Next Steps

On PyPi:
* [py_trees](https://pypi.org/project/py-trees/)
* [py_trees_js](https://pypi.org/project/py-trees-js/)

Examples:
* [ReadTheDocs - PyTrees ROS Tutorials](https://py-trees-ros-tutorials.readthedocs.io/en/devel/index.html) - significantly more edifying than the demos, these incrementally walk through the process of building a decision making layer for a robot. These use ROS2 (sparsely), but merely browsing should be enlightening regardless.

Visualisation:
* [py_trees_js](https://github.com/splintered-reality/py_trees_js) - a javascript library for building your own runtime visualisation tool

Robotics:
* [py_trees_ros](https://github.com/splintered-reality/py_trees_ros) - a tree manager and behaviours designed for use specifically with `ROS2`
* [py_trees_ros_viewer](https://github.com/splintered-reality/py_trees_ros_viewer) - a `Qt/ROS2` implementation of `py_trees_js`

## Acknowledgments

`py_trees` has grown thanks to contributions from the community.

### Typed input/output ports (2026)

Port declarations, type validation, and XML parser. Contributed by [Sunrise Robotics](https://sunriserobotics.co/). Work performed by Jennifer Buehler ([@JenniferBuehler](https://github.com/JenniferBuehler)) while engaged at Sunrise Robotics.

### 2.4.x — ForEach, CompareBlackboardVariables, callables in comparisons (2025)

* **`ForEach` decorator** — Jorge Santos Simón
* **`CompareBlackboardVariables` behaviour** — Jorge Santos Simón
* **Callables in `ComparisonExpression`** — Jorge Santos Simón
* **`typing.Any` for kwargs type hints, release management** — Sebastian Castro ([@sea-bass](https://github.com/sea-bass))
* **Replace type checks with instance checks** — FrankvVeelen ([@FrankvVeelen](https://github.com/FrankvVeelen))
* **Short-circuit evaluation in `all()` via generator expressions** — Lei Zhang
* **Remove `py` package (pytest-benchmark)** — Robert Tang-Kong ([@tangkong](https://github.com/tangkong))
* **Docs typo fix** — Sid Sarasvati

### 2.3.x — Python 3.12 support, ProbabilisticBehaviour (2025)

* **Python 3.12 support, CI modernisation, mypy fixes, tick-tock improvements** — Sebastian Castro ([@sea-bass](https://github.com/sea-bass))
* **`ProbabilisticBehaviour`** — [@gitpushoriginmaster](https://github.com/gitpushoriginmaster)
* **Parallel `SuccessOnOne` short-circuit fix** — wanfeng ([@starrye](https://github.com/starrye))
* **`typing.Sequence` for composite `children` argument** — Efe Mert Demir
* **Timer duration type-check fix** — Naveed Usmani
* **Docs: ticking tree, make target fix** — Nino Walker
* **Fixed broken examples** — [@Tunglies](https://github.com/Tunglies)
* **Dev environment polish (VSCode extensions, devcontainers, README)** — Daniel Stonier ([@stonier](https://github.com/stonier))

### 2.2.x — Sequences & selectors with/without memory (2023)

* **Sequences and selectors with and without memory, `Repeat`/`Retry` decorators, explicit composite/behaviour arguments, poetry + tox + mypy + black/usort, new abstract base classes, `StatusQueue`** — Daniel Stonier ([@stonier](https://github.com/stonier))
* **Display bugfix for off-the-grid blackboard client nodes, exclusive-write edge styling** — Matt Sprague
* **Docs: `py_trees.common.Access` added to modules** — Jan-Hendrik Ewers
* **Typos** — Liang Fok
* **Tutorial URL update** — Andrew Best ([@andrewbest-tri](https://github.com/andrewbest-tri))

### 2.1.x — Chooser deprecated, API housekeeping (2020–2021)

* **Chain-able `add_children()` for composites** — [@iovsiann](https://github.com/iovsiann)
* **Bugfix: current-child setting while moving through children** — Sara Sinback
* **Blackboard fix** — Gabriel Rodrigues
* **`Chooser` deprecation, `either_or` idiom, `TickCounter`, `CheckBlackboardVariableValues`, `ComparisonExpression`, mypy conformance** — Daniel Stonier ([@stonier](https://github.com/stonier))

### 2.0.x — Blackboards V2 (2019)

* **Blackboards V2: namespaced clients, read/write access control, exclusive-write access, key remappings, activity stream, required keys, `Access` enum, `DisplaySnapshotVisitor`** — Daniel Stonier ([@stonier](https://github.com/stonier))
* **`show_only_visited` indentation fix** — [@andrewbest-tri](https://github.com/andrewbest-tri)
* **Additional Parallel behaviour tests** — Sachit Vithaldas ([@sachit-tri](https://github.com/sachit-tri))

### 1.2.x — `shutdown()`, `EternalGuard`, `StatusToBlackboard` (2019)

* **`Behaviour.shutdown()`, `EternalGuard` decorator + idiom, `StatusToBlackboard` decorator, `xhtml_tree` display** — Daniel Stonier ([@stonier](https://github.com/stonier))
* **`Visitor.finalise()` method** — Jan Feil ([@jfeil](https://github.com/jfeil))
* **Timer licence header fix** — Naveed Usmani

### 1.0.x — New-style decorators (2019)

* **New-style decorators in `py_trees.decorators`, tip-in-ascii-tree highlighting, flexible `setup()` via kwargs, `add_visitor()`** — Daniel Stonier ([@stonier](https://github.com/stonier))
* **Unicode-char fallbacks when stdout is not UTF-8, synchronous tree setup on infinite timeout, double-edge fix, section labels** — Sachit Vithaldas ([@sachit-tri](https://github.com/sachit-tri))
* **`Blackboard.unset()`, parent field correctly set in `Behaviour.replace_child()`** — Kevin Greene
* **Windows compatibility (keypress handling)** — Fredrik Baberg
* **Documentation / code typo fixes** — Chris Lalancette

### 0.x — Foundations (2015–2018)

* **Initial implementation of behaviour trees in Python, `OneShot`, `PickUpWhereYouLeftOff` idioms, ROS 2 compatibility, dot-tree/ASCII-tree display, visitors and generators, ComparisonExpression, namespaces consolidation** — Daniel Stonier ([@stonier](https://github.com/stonier))
* **Blackboard watcher/monitor, `SubBlackboard`, subscriber behaviours, `Imposter` tick fixes, meta `add_child()`, license / docstring cleanup** — Naveed Usmani
* **`gopher_behaviours`: elevator transfer & autonomous rides (field-test-era behaviours)** — Marcus Liebhardt
* **Maintainer handover & project metadata** — Michal Staniaszek
* **Python 2/3 compatibility** — Aadesh Neupane
* **Improved error logging** — Alex V
* **Test/doc dependency isolation from installed deps, pytest fix for issue #80** — Sachit Vithaldas ([@sachit-tri](https://github.com/sachit-tri))
* **Early maintenance & packaging** — dwlee, Alexander Reimann

### Maintainers

Daniel Stonier ([@stonier](https://github.com/stonier)), Naveed Usmani, Michal Staniaszek, Sebastian Castro ([@sea-bass](https://github.com/sea-bass)).

## Releases

* `2.4.x` - New ForEach decorator, new CompareBlackboardVariables behaviour, and callables in comparison expressions.
* `2.3.x` - Support for Python 3.12 was added, and Python 3.8 was dropped.
* `2.2.x` - Selectors, Sequences with and without memory. Improved testing and style/type checking.
* `2.1.x` - Chooser deprecated. API housekeeping.
* `2.0.x` - Blackboards V2!
* `1.2.x` - Trees can now shutdown cleanly. StatusToBlackboard and EternalGuard, Visitors get finalise().
* `1.1.x` - Fixes for setup, tick-tock, viz.
* `1.0.x` - Behaviours, Decorators, Composites, Blackboards, Tree Management and Viz tools.
* `0.y.x` - First open source pre-releases.

| | Devel | 2.4.x | 2.3.x | 2.2.x | 2.1.x | 2.0.x |
|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
| Sources | [![devel][sources-devel-image]][sources-devel] | [![2.4.x][sources-2.4.x-image]][sources-2.4.x] | [![2.3.x][sources-2.3.x-image]][sources-2.3.x] | [![2.2.x][sources-2.2.x-image]][sources-2.2.x] | [![2.1.x][sources-2.1.x-image]][sources-2.1.x] | [![2.0.x][sources-2.0.x-image]][sources-2.0.x] |
| Compatibility | [![Python 3.12][python312-image]][python312-docs]<br/>[![Python 3.10][python310-image]][python310-docs] | [![Python 3.12][python312-image]][python312-docs]<br/>[![Python 3.10][python310-image]][python310-docs] | [![Python 3.12][python312-image]][python312-docs]<br/>[![Python 3.10][python310-image]][python310-docs] | [![Python 3.10][python310-image]][python310-docs]<br/>[![Python 3.8][python38-image]][python38-docs] | [![Python 3.6][python36-image]][python36-docs] | [![Python 3.6][python36-image]][python36-docs] | [![Python 3.6][python36-image]][python36-docs] |
| CI | [![devel-Status][devel-build-status-image]][devel-build-status] | [![2.4.x-Status][2.4.x-build-status-image]][2.4.x-build-status] | [![2.3.x-Status][2.3.x-build-status-image]][2.3.x-build-status] | [![2.2.x-Status][2.2.x-build-status-image]][2.2.x-build-status] | - | - | - | - |
| Documentation | [![devel-Docs][rtd-devel-image]][docs-devel] | [![2.4.x-Docs][rtd-2.4.x-image]][docs-2.4.x] | [![2.3.x-Docs][rtd-2.3.x-image]][docs-2.3.x] | [![2.2.x-Docs][rtd-2.2.x-image]][docs-2.2.x] | [![2.1.x-Docs][rtd-2.1.x-image]][docs-2.1.x] | [![2.0.x-Docs][rtd-2.0.x-image]][docs-2.0.x] |


[license-image]: https://img.shields.io/badge/License-BSD%203--Clause-orange.svg?style=plastic
[license]: LICENSE

[python312-image]: https://img.shields.io/badge/python-3.12-green.svg?style=plastic
[python312-docs]: https://docs.python.org/3.12/
[python310-image]: https://img.shields.io/badge/python-3.10-green.svg?style=plastic
[python310-docs]: https://docs.python.org/3.10/
[python38-image]: https://img.shields.io/badge/python-3.8-green.svg?style=plastic
[python38-docs]: https://docs.python.org/3.8/
[python36-image]: https://img.shields.io/badge/python-3.6-green.svg?style=plastic
[python36-docs]: https://docs.python.org/3.6/

[devel-build-status-image]: https://github.com/splintered-reality/py_trees/actions/workflows/pre-merge.yaml/badge.svg
[devel-build-status]: https://github.com/splintered-reality/py_trees/actions/workflows/pre-merge.yaml
[2.4.x-build-status-image]: https://github.com/splintered-reality/py_trees/actions/workflows/pre-merge.yaml/badge.svg??branch=release/2.4.x
[2.4.x-build-status]: https://github.com/splintered-reality/py_trees/actions/workflows/pre-merge.yaml
[2.3.x-build-status-image]: https://github.com/splintered-reality/py_trees/actions/workflows/pre-merge.yaml/badge.svg??branch=release/2.3.x
[2.3.x-build-status]: https://github.com/splintered-reality/py_trees/actions/workflows/pre-merge.yaml
[2.2.x-build-status-image]: https://github.com/splintered-reality/py_trees/actions/workflows/pre-merge.yaml/badge.svg??branch=release/2.2.x
[2.2.x-build-status]: https://github.com/splintered-reality/py_trees/actions/workflows/pre-merge.yaml

[docs-devel]: http://py-trees.readthedocs.io/
[docs-2.4.x]: http://py-trees.readthedocs.io/en/release-2.4.x/
[docs-2.3.x]: http://py-trees.readthedocs.io/en/release-2.3.x/
[docs-2.2.x]: http://py-trees.readthedocs.io/en/release-2.2.x/
[docs-2.1.x]: http://py-trees.readthedocs.io/en/release-2.1.x/
[docs-2.0.x]: http://py-trees.readthedocs.io/en/release-2.0.x/
[docs-1.3.x]: http://py-trees.readthedocs.io/en/release-1.3.x/
[docs-1.2.x]: http://py-trees.readthedocs.io/en/release-1.2.x/
[docs-0.7.x]: http://py-trees.readthedocs.io/en/release-0.7.x/
[docs-0.6.x]: http://py-trees.readthedocs.io/en/release-0.6.x/
[docs-0.5.x]: http://docs.ros.org/kinetic/api/py_trees/html/

[docs-devel-image]: http://img.shields.io/badge/docs-devel-brightgreen.svg?style=plastic
[docs-2.4.x-image]: http://img.shields.io/badge/docs-2.4.x-brightgreen.svg?style=plastic
[docs-2.3.x-image]: http://img.shields.io/badge/docs-2.3.x-brightgreen.svg?style=plastic
[docs-2.2.x-image]: http://img.shields.io/badge/docs-2.2.x-brightgreen.svg?style=plastic
[docs-2.1.x-image]: http://img.shields.io/badge/docs-2.1.x-brightgreen.svg?style=plastic
[docs-2.0.x-image]: http://img.shields.io/badge/docs-2.0.x-brightgreen.svg?style=plastic
[docs-1.3.x-image]: http://img.shields.io/badge/docs-1.3.x-brightgreen.svg?style=plastic
[docs-1.2.x-image]: http://img.shields.io/badge/docs-1.2.x-brightgreen.svg?style=plastic
[docs-0.7.x-image]: http://img.shields.io/badge/docs-0.7.x-brightgreen.svg?style=plastic
[docs-0.6.x-image]: http://img.shields.io/badge/docs-0.6.x-brightgreen.svg?style=plastic
[docs-0.5.x-image]: http://img.shields.io/badge/docs-0.5.x-brightgreen.svg?style=plastic

[rtd-devel-image]: https://readthedocs.org/projects/py-trees/badge/?version=devel&style=plastic
[rtd-2.4.x-image]: https://readthedocs.org/projects/py-trees/badge/?version=release-2.4.x&style=plastic
[rtd-2.3.x-image]: https://readthedocs.org/projects/py-trees/badge/?version=release-2.3.x&style=plastic
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
[sources-2.4.x]: https://github.com/splintered-reality/py_trees/tree/release/2.4.x
[sources-2.3.x]: https://github.com/splintered-reality/py_trees/tree/release/2.3.x
[sources-2.2.x]: https://github.com/splintered-reality/py_trees/tree/release/2.2.x
[sources-2.1.x]: https://github.com/splintered-reality/py_trees/tree/release/2.1.x
[sources-2.0.x]: https://github.com/splintered-reality/py_trees/tree/release/2.0.x
[sources-1.3.x]: https://github.com/splintered-reality/py_trees/tree/release/1.3.x
[sources-1.2.x]: https://github.com/splintered-reality/py_trees/tree/release/1.2.x
[sources-0.7.x]: https://github.com/splintered-reality/py_trees/tree/release/0.7.x
[sources-0.6.x]: https://github.com/splintered-reality/py_trees/tree/release/0.6.x
[sources-0.5.x]: https://github.com/splintered-reality/py_trees/tree/release/0.5.x

[sources-devel-image]: http://img.shields.io/badge/sources-devel-blue.svg?style=plastic
[sources-2.4.x-image]: http://img.shields.io/badge/sources-2.4.x-blue.svg?style=plastic
[sources-2.3.x-image]: http://img.shields.io/badge/sources-2.3.x-blue.svg?style=plastic
[sources-2.2.x-image]: http://img.shields.io/badge/sources-2.2.x-blue.svg?style=plastic
[sources-2.1.x-image]: http://img.shields.io/badge/sources-2.1.x-blue.svg?style=plastic
[sources-2.0.x-image]: http://img.shields.io/badge/sources-2.0.x-blue.svg?style=plastic
[sources-1.3.x-image]: http://img.shields.io/badge/sources-1.3.x-blue.svg?style=plastic
[sources-1.2.x-image]: http://img.shields.io/badge/sources-1.2.x-blue.svg?style=plastic
[sources-0.7.x-image]: http://img.shields.io/badge/sources-0.7.x-blue.svg?style=plastic
[sources-0.6.x-image]: http://img.shields.io/badge/sources-0.6.x-blue.svg?style=plastic
[sources-0.5.x-image]: http://img.shields.io/badge/sources-0.5.x-blue.svg?style=plastic
