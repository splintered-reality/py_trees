#!/usr/bin/env python3
#
# License: BSD
#   https://raw.githubusercontent.com/splintered-reality/py_trees/devel/LICENSE
#

##############################################################################
# Imports
##############################################################################

import time
import typing

import py_trees
import py_trees.console as console

##############################################################################
# Helpers
##############################################################################


class Motley(object):
    """
    To test nested access on the blackboard
    """

    def __init__(self) -> None:
        self.nested = "nested"


class create_blackboards(object):
    def __enter__(
        self,
    ) -> typing.Tuple[py_trees.blackboard.Client, py_trees.blackboard.Client]:
        self.root = py_trees.blackboard.Client(name="Root")
        self.parameters = py_trees.blackboard.Client(
            name="Namespaced", namespace="parameters"
        )
        return (self.root, self.parameters)

    def __exit__(
        self,
        unused_type: typing.Any,
        unused_value: typing.Any,
        unused_traceback: typing.Any,
    ) -> None:
        self.parameters.unregister(clear=True)
        self.root.unregister(clear=True)


##############################################################################
# Tests
##############################################################################


def benchmark_registration() -> None:
    console.banner("Registration Benchmarks (x1000)")
    start_time = None
    print("No Remaps")
    with create_blackboards() as (root, parameters):
        width = max(len(root.name), len(parameters.name), len("Unregister"))
        for blackboard in (root, parameters):
            start_time = time.monotonic()
            for i in range(0, 1000):
                blackboard.register_key(
                    key=str(i),
                    access=py_trees.common.Access.READ,
                )
            duration = time.monotonic() - start_time
            print(
                " - "
                + console.cyan
                + "{0: <{1}}".format(blackboard.name, width)
                + console.reset
                + ": "
                + console.yellow
                + "{0:.3f}".format(duration)
                + console.reset
            )
        start_time = time.monotonic()
    duration = time.monotonic() - start_time
    print(
        " - "
        + console.cyan
        + "{0: <{1}}".format("Unregister", width)
        + console.reset
        + ": "
        + console.yellow
        + "{0:.3f}".format(duration)
        + console.reset
    )

    with create_blackboards() as (root, parameters):
        remaps = {i: "/state/{}".format(i) for i in range(0, 1000)}
        print("Remaps (x1000)")
        for blackboard in (root, parameters):
            start_time = time.monotonic()
            for i in range(0, 1000):
                blackboard.register_key(
                    key=str(i), access=py_trees.common.Access.READ, remap_to=remaps[i]
                )
            duration = time.monotonic() - start_time
            print(
                " - "
                + console.cyan
                + "{0: <{1}}".format(blackboard.name, width)
                + console.reset
                + ": "
                + console.yellow
                + "{0:.3f}".format(duration)
                + console.reset
            )
        start_time = time.monotonic()
    duration = time.monotonic() - start_time
    print(
        " - "
        + console.cyan
        + "{0: <{1}}".format("Unregister", width)
        + console.reset
        + ": "
        + console.yellow
        + "{0:.3f}".format(duration)
        + console.reset
    )


def benchmark_read() -> None:
    console.banner("Read Benchmarks (x1000)")

    def _impl(with_remaps: bool = False) -> None:
        with create_blackboards() as (root, parameters):
            width = max(len(root.name), len(parameters.name))
            for blackboard in (root, parameters):
                for i in range(0, 1000):
                    if with_remaps:
                        remaps = {
                            i: "/state/{}/colander_{}".format(
                                blackboard.name.lower(), i
                            )
                            for i in range(0, 1000)
                        }
                        suffix = " with Remaps"
                        blackboard.register_key(
                            key="colander_{}".format(i),
                            access=py_trees.common.Access.READ,
                            remap_to=remaps[i],
                        )
                    else:
                        suffix = ""
                        blackboard.register_key(
                            key="colander_{}".format(i),
                            access=py_trees.common.Access.READ,
                        )
            for i in range(0, 1000):
                if with_remaps:
                    for blackboard in (root, parameters):
                        py_trees.blackboard.Blackboard.set(
                            "/state/{}/colander_{}".format(blackboard.name.lower(), i),
                            i,
                        )
                else:
                    py_trees.blackboard.Blackboard.set("/colander_{}".format(i), i)
                    py_trees.blackboard.Blackboard.set(
                        "/parameters/colander_{}".format(i), i
                    )
            relative_names = {}
            absolute_names: typing.Dict[str, typing.Dict[int, str]] = {
                "Root": {},
                "Namespaced": {},
            }
            for i in range(0, 1000):
                relative_names[i] = "colander_{}".format(i)
                absolute_names["Root"][i] = "/colander_{}".format(i)
                absolute_names["Namespaced"][i] = "/parameters/colander_{}".format(i)
            print("Relative Names" + suffix)
            for blackboard in (root, parameters):
                start_time = time.monotonic()
                foo = 0
                for i in range(0, 1000):
                    foo += blackboard.get(relative_names[i])
                duration = time.monotonic() - start_time
                print(
                    " - "
                    + console.cyan
                    + "{0: <{1}}".format(blackboard.name, width)
                    + console.reset
                    + ": "
                    + console.yellow
                    + "{0:.3f}".format(duration)
                    + console.reset
                )
            print("Absolute Names" + suffix)
            for blackboard in (root, parameters):
                names = absolute_names[blackboard.name]
                start_time = time.monotonic()
                foo = 0
                for i in range(0, 1000):
                    foo += blackboard.get(names[i])
                duration = time.monotonic() - start_time
                print(
                    " - "
                    + console.cyan
                    + "{0: <{1}}".format(blackboard.name, width)
                    + console.reset
                    + ": "
                    + console.yellow
                    + "{0:.3f}".format(duration)
                    + console.reset
                )

    _impl(with_remaps=False)
    # _impl(with_remaps=True)


def benchmark_write() -> None:
    console.banner("Write Benchmarks (x1000)")

    def _impl(with_remaps: bool = False) -> None:
        with create_blackboards() as (root, parameters):
            width = max(len(root.name), len(parameters.name))
            for blackboard in (root, parameters):
                for i in range(0, 1000):
                    if with_remaps:
                        remaps = {
                            i: "/state/{}/{}".format(blackboard.name.lower(), i)
                            for i in range(0, 1000)
                        }
                        suffix = " with Remaps"
                        blackboard.register_key(
                            key=str(i),
                            access=py_trees.common.Access.WRITE,
                            remap_to=remaps[i],
                        )
                    else:
                        suffix = ""
                        blackboard.register_key(
                            key=str(i),
                            access=py_trees.common.Access.WRITE,
                        )
            relative_names = {}
            absolute_names: typing.Dict[str, typing.Dict[int, str]] = {
                "Root": {},
                "Namespaced": {},
            }
            for i in range(0, 1000):
                relative_names[i] = str(i)
                absolute_names["Root"][i] = "/{}".format(str(i))
                absolute_names["Namespaced"][i] = "/parameters/{}".format(str(i))
            print("Relative Names" + suffix)
            for blackboard in (root, parameters):
                start_time = time.monotonic()
                foo = 0
                for i in range(0, 1000):
                    foo += blackboard.set(relative_names[i], i)
                duration = time.monotonic() - start_time
                print(
                    " - "
                    + console.cyan
                    + "{0: <{1}}".format(blackboard.name, width)
                    + console.reset
                    + ": "
                    + console.yellow
                    + "{0:.3f}".format(duration)
                    + console.reset
                )
            print("Absolute Names" + suffix)
            for blackboard in (root, parameters):
                names = absolute_names[blackboard.name]
                start_time = time.monotonic()
                foo = 0
                for i in range(0, 1000):
                    foo += blackboard.set(names[i], i)
                duration = time.monotonic() - start_time
                print(
                    " - "
                    + console.cyan
                    + "{0: <{1}}".format(blackboard.name, width)
                    + console.reset
                    + ": "
                    + console.yellow
                    + "{0:.3f}".format(duration)
                    + console.reset
                )

    _impl(with_remaps=False)
    _impl(with_remaps=True)


##############################################################################
# Main
##############################################################################

# To profile this code:
#   python -m cProfile -s cumtime benchmark_blackboard.py


if __name__ == "__main__":
    benchmark_registration()
    benchmark_read()
    benchmark_write()
