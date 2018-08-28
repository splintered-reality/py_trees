#!/usr/bin/env python
#
# License: BSD
#   https://raw.githubusercontent.com/stonier/py_trees/devel/LICENSE
#
##############################################################################
# Imports
##############################################################################

# enable some python3 compatibility options:
# (unicode_literals not compatible with python2 uuid module)
from __future__ import absolute_import, print_function

import py_trees
import py_trees.console as console

##############################################################################
# Logging Level
##############################################################################

py_trees.logging.level = py_trees.logging.Level.DEBUG
logger = py_trees.logging.Logger("Nosetest")


##############################################################################
# Helpers
##############################################################################


def tick_tree(root):
    """
    Tick the tree and return the node count on the last tick.
    @returns count : the number of nodes traversed on the last tick
    """
    print("\n================== Iteration 1-2 ==================\n")
    for i in range(1, 3):
        count = 0
        print(("\n--------- Run %s ---------\n" % i))
        for unused_node in root.tick():
            count += 1
    return count


@py_trees.meta.oneshot
class OneShotSequence(py_trees.composites.Sequence):
    pass


class CounterTester(py_trees.behaviours.Count):
    def __init__(self, *args, **kwargs):
        super(CounterTester, self).__init__(*args, **kwargs)
        self.terminate_count = 0
        self.initialise_count = 0

    def terminate(self, new_status):
        super(CounterTester, self).terminate(new_status)
        self.terminate_count += 1

    def initialise(self):
        super(CounterTester, self).initialise()
        self.initialise_count += 1

##############################################################################
# Tests
##############################################################################


def test_oneshot_does_not_modify_class():
    print(console.bold + "\n****************************************************************************************" + console.reset)
    print(console.bold + "* Test Oneshots Do Not Modify the Original Class" + console.reset)
    print(console.bold + "****************************************************************************************\n" + console.reset)
    oneshot = py_trees.meta.oneshot(py_trees.composites.Sequence)(name="Oneshot",
                                                                  children=[py_trees.behaviours.Failure(name="Failure")])
    oneshot_count = tick_tree(oneshot)
    print(console.yellow + "\nOneshot Sequence Tick Count: {}".format(oneshot_count) + console.reset)

    decorated_oneshot = OneShotSequence(name="Oneshot",
                                        children=[py_trees.behaviours.Failure(name="Failure")])
    decorated_oneshot_count = tick_tree(decorated_oneshot)
    print(console.yellow + "\nDecorated Oneshot Sequence Tick Count: {}".format(decorated_oneshot_count) + console.reset)

    normal = py_trees.composites.Sequence(name="Normal",
                                          children=[py_trees.behaviours.Failure(name="Failure")])
    normal_count = tick_tree(normal)
    print(console.yellow + "\nNormal Sequence Tick Count: {}".format(normal_count) + console.reset)
    assert(oneshot_count == 1)
    assert(decorated_oneshot_count == 1)
    # Hitherto, the decorator modified the sequence permanently, so the tick count for the latter
    # would be the same. This behaviour is confusing - the decorators should always create
    # *new* classes so there is only one policy for decoration - instance modification only.
    assert(normal_count == 2)


def test_oneshot_does_not_re_initialise():
    """Makes sure that the oneshot decorator does not call terminate or
    initialise more than once in case of valid completion of the behaviour
    involving a SUCCESS new_status.
    """
    success = py_trees.behaviours.Success(name="Success")
    tester = py_trees.meta.oneshot(CounterTester)(
        name="Tester",
        fail_until=0,
        running_until=0,
        success_until=1000,
        reset=False)
    root = py_trees.composites.Sequence(
        name="Sequence", children=[success, tester])
    for i in range(5):
        root.tick_once()

    assert tester.status == py_trees.common.Status.SUCCESS
    print("Terminate count {}".format(tester.terminate_count))
    print("Initialize count {}".format(tester.initialise_count))
    assert tester.terminate_count == 1
    assert tester.initialise_count == 1


def test_oneshot_does_re_initialise():
    """Makes sure that the oneshot decorator calls terminate and
    initialise if they were called due to early termination involving
    an INVALID new_status. Also ensures that it does not call it
    after it reports SUCCESS.
    """
    NUM_TICKS = 10
    periodic_success = py_trees.behaviours.SuccessEveryN(
        name="Periodic_Fail", n=2)
    tester = py_trees.meta.oneshot(CounterTester)(
        name="Tester",
        fail_until=0,
        running_until=2,
        success_until=1000,
        reset=False)
    root = py_trees.composites.Selector(
        name="Selector", children=[
            periodic_success, tester])

    # Alternating Failure and Success.
    periodic_expected_status = [
        py_trees.common.Status.SUCCESS
        if (n % 2 == 1) else py_trees.common.Status.FAILURE for n in range(NUM_TICKS)]

    tester_expected_status = [py_trees.common.Status.RUNNING,  # First initialise()
                              py_trees.common.Status.INVALID,  # First terminate()
                              py_trees.common.Status.RUNNING,  # Second initialise()
                              py_trees.common.Status.INVALID,  # Second terminate()
                              py_trees.common.Status.SUCCESS,  # Third initialise() and terminate()
                              py_trees.common.Status.SUCCESS,
                              py_trees.common.Status.SUCCESS,
                              py_trees.common.Status.SUCCESS,
                              py_trees.common.Status.SUCCESS,
                              py_trees.common.Status.SUCCESS
                              ]

    root_expected_status = [py_trees.common.Status.RUNNING,
                            py_trees.common.Status.SUCCESS,
                            py_trees.common.Status.RUNNING,
                            py_trees.common.Status.SUCCESS,
                            py_trees.common.Status.SUCCESS,
                            py_trees.common.Status.SUCCESS,
                            py_trees.common.Status.SUCCESS,
                            py_trees.common.Status.SUCCESS,
                            py_trees.common.Status.SUCCESS,
                            py_trees.common.Status.SUCCESS,
                            ]

    for i in range(NUM_TICKS):
        root.tick_once()
        assert root.status == root_expected_status[i]
        assert tester.status == tester_expected_status[i]
        assert periodic_success.status == periodic_expected_status[i]

    print("Terminate count {}".format(tester.terminate_count))
    print("Initialize count {}".format(tester.initialise_count))
    assert tester.terminate_count == 3
    assert tester.initialise_count == 3
