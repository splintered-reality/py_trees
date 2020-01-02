#!/usr/bin/env python
#
# License: BSD
#   https://raw.githubusercontent.com/splintered-reality/py_trees/devel/LICENSE
#

##############################################################################
# Imports
##############################################################################

import nose
import time

import py_trees
import py_trees.console as console

##############################################################################
# Helpers
##############################################################################


class Motley(object):
    """
    To test nested access on the blackboard
    """
    def __init__(self):
        self.nested = "nested"


class create_blackboards(object):

    def __enter__(self):
        self.root = py_trees.blackboard.Client(
            name="Root"
        )
        self.parameters = py_trees.blackboard.Client(
            name="Namespaced",
            namespace="parameters"
        )
        return (self.root, self.parameters)

    def __exit__(self, unused_type, unused_value, unused_traceback):
        self.parameters.unregister(clear=True)
        self.root.unregister(clear=True)


##############################################################################
# Tests
##############################################################################


def test_registration():
    console.banner("Registration Benchmarks (x1000)")
    print("No Remaps")
    with create_blackboards() as (root, parameters):
        width = max(len(root.name), len(parameters.name))
        for blackboard in (root, parameters):
            start_time = time.monotonic()
            for i in range(0, 1000):
                blackboard.register_key(
                    key=str(i),
                    access=py_trees.common.Access.READ,
                )
            duration = time.monotonic() - start_time
            print(" - " + console.cyan + "{0: <{1}}".format(blackboard.name, width) + console.reset + ": " + console.yellow + "{0:.3f}".format(duration) + console.reset)

    with create_blackboards() as (root, parameters):
        remaps = {i: "/state/{}".format(i) for i in range(0, 1000)}
        print("Remaps (x1000)")
        for blackboard in (root, parameters):
            start_time = time.monotonic()
            for i in range(0, 1000):
                blackboard.register_key(
                    key=str(i),
                    access=py_trees.common.Access.READ,
                    remap_to=remaps[i]
                )
            duration = time.monotonic() - start_time
            print(" - " + console.cyan + "{0: <{1}}".format(blackboard.name, width) + console.reset + ": " + console.yellow + "{0:.3f}".format(duration) + console.reset)
    assert(True)


def test_read():
    console.banner("Read Benchmarks (x1000)")

    def _impl(with_remaps=False):
        with create_blackboards() as (root, parameters):
            width = max(len(root.name), len(parameters.name))
            for blackboard in (root, parameters):
                for i in range(0, 1000):
                    if with_remaps:
                        remaps = {i: "/state/{}/{}".format(blackboard.name.lower(), i) for i in range(0, 1000)}
                        suffix = " with Remaps"
                        blackboard.register_key(
                            key=str(i),
                            access=py_trees.common.Access.READ,
                            remap_to=remaps[i]
                        )
                    else:
                        suffix = ""
                        blackboard.register_key(
                            key=str(i),
                            access=py_trees.common.Access.READ,
                        )
            for i in range(0, 1000):
                if with_remaps:
                    for blackboard in (root, parameters):
                        py_trees.blackboard.Blackboard.set("/state/{}/{}".format(blackboard.name.lower(), i), i)
                else:
                    py_trees.blackboard.Blackboard.set("/{}".format(i), i)
                    py_trees.blackboard.Blackboard.set("/parameters/{}".format(i), i)
            relative_names = {}
            absolute_names = {"Root": {}, "Namespaced": {}}
            for i in range(0, 1000):
                relative_names[i] = str(i)
                absolute_names["Root"][i] = "/{}".format(str(i))
                absolute_names["Namespaced"][i] = "/parameters/{}".format(str(i))
            print("Relative Names" + suffix)
            for blackboard in (root, parameters):
                start_time = time.monotonic()
                foo = 0
                for i in range(0, 1000):
                    foo += blackboard.get(relative_names[i])
                duration = time.monotonic() - start_time
                print(" - " + console.cyan + "{0: <{1}}".format(blackboard.name, width) + console.reset + ": " + console.yellow + "{0:.3f}".format(duration) + console.reset)
            print("Absolute Names" + suffix)
            for blackboard in (root, parameters):
                names = absolute_names[blackboard.name]
                start_time = time.monotonic()
                foo = 0
                for i in range(0, 1000):
                    foo += blackboard.get(names[i])
                duration = time.monotonic() - start_time
                print(" - " + console.cyan + "{0: <{1}}".format(blackboard.name, width) + console.reset + ": " + console.yellow + "{0:.3f}".format(duration) + console.reset)
    _impl(with_remaps=False)
    _impl(with_remaps=True)
    assert(True)


def test_write():
    console.banner("Write Benchmarks (x1000)")

    def _impl(with_remaps=False):
        with create_blackboards() as (root, parameters):
            width = max(len(root.name), len(parameters.name))
            for blackboard in (root, parameters):
                for i in range(0, 1000):
                    if with_remaps:
                        remaps = {i: "/state/{}/{}".format(blackboard.name.lower(), i) for i in range(0, 1000)}
                        suffix = " with Remaps"
                        blackboard.register_key(
                            key=str(i),
                            access=py_trees.common.Access.WRITE,
                            remap_to=remaps[i]
                        )
                    else:
                        suffix = ""
                        blackboard.register_key(
                            key=str(i),
                            access=py_trees.common.Access.WRITE,
                        )
            relative_names = {}
            absolute_names = {"Root": {}, "Namespaced": {}}
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
                print(" - " + console.cyan + "{0: <{1}}".format(blackboard.name, width) + console.reset + ": " + console.yellow + "{0:.3f}".format(duration) + console.reset)
            print("Absolute Names" + suffix)
            for blackboard in (root, parameters):
                names = absolute_names[blackboard.name]
                start_time = time.monotonic()
                foo = 0
                for i in range(0, 1000):
                    foo += blackboard.set(names[i], i)
                duration = time.monotonic() - start_time
                print(" - " + console.cyan + "{0: <{1}}".format(blackboard.name, width) + console.reset + ": " + console.yellow + "{0:.3f}".format(duration) + console.reset)
    _impl(with_remaps=False)
    _impl(with_remaps=True)
    assert(True)
