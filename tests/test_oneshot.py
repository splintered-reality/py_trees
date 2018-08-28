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


def tick_tree(root, oneshot, from_tick, to_tick):
    """
    Tick the tree and return the node count on the last tick.

    Args:
        root (:class:`~py_trees.behaviour.Behaviour`): root of the tree
        oneshot (:class:`~py_trees.behaviour.Behaviour`): oneshot behaviour to track
        from_tick (:obj:`int`) : starting tick number
        to_tick (:obj:`int`) : final tick number (inclusive)

    Returns:
        count : the number of nodes traversed on the last tick
    """
    print("\n================== Iterations {}-{} ==================\n".format(from_tick, to_tick))
    for i in range(from_tick, to_tick + 1):
        count = 0
        print(("\n--------- Run %s ---------\n" % i))
        for unused_node in root.tick():
            count += 1
        if oneshot:
            oneshot.logger.debug("OneShot.status[{}[{}]]".format(oneshot.status, oneshot.original.status))
    return count


class Foo(py_trees.behaviour.Behaviour):

    def __init__(self, name, what_am_i):
        """
        Constructor with the usual behaviour name argument plus
        a custom argument.

        Args:
            name (:obj:`str`): the behaviour name
            name (:obj:`str`): descriptive token for the behaviour
        """
        super(Foo, self).__init__(name)
        self.what_am_i = what_am_i


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


def test_custom_construction():
    print(console.bold + "\n****************************************************************************************" + console.reset)
    print(console.bold + "* Construction Arguments are passed" + console.reset)
    print(console.bold + "****************************************************************************************\n" + console.reset)
    oneshot = py_trees.meta.oneshot(Foo)(name="Oneshot",
                                         what_am_i="Bar")
    print(console.cyan + "What Am I: " + console.yellow + oneshot.what_am_i + console.reset)
    assert(oneshot.what_am_i, "Bar")


def test_oneshot_does_not_modify_class():
    print(console.bold + "\n****************************************************************************************" + console.reset)
    print(console.bold + "* Test Oneshots Do Not Modify the Original Class" + console.reset)
    print(console.bold + "****************************************************************************************\n" + console.reset)
    oneshot = py_trees.meta.oneshot(py_trees.composites.Sequence)(name="Oneshot",
                                                                  children=[py_trees.behaviours.Failure(name="Failure")])
    oneshot_count = tick_tree(root=oneshot,
                              oneshot=oneshot,
                              from_tick=1,
                              to_tick=2)
    print(console.cyan + "\nOneshot Sequence Tick Count: " + console.yellow + "{}".format(oneshot_count) + console.reset)
    assert(oneshot_count == 1)

    decorated_oneshot = OneShotSequence(
        name="Oneshot",
        children=[py_trees.behaviours.Failure(name="Failure")])
    decorated_oneshot_count = tick_tree(root=decorated_oneshot,
                                        oneshot=decorated_oneshot,
                                        from_tick=1,
                                        to_tick=2)
    print(console.cyan + "\nDecorated Oneshot Sequence Tick Count: " + console.yellow + "{}".format(decorated_oneshot_count) + console.reset)
    assert(decorated_oneshot_count == 1)

    normal = py_trees.composites.Sequence(
        name="Normal",
        children=[py_trees.behaviours.Failure(name="Failure")])
    normal_count = tick_tree(root=normal, oneshot=None, from_tick=1, to_tick=2)
    print(console.cyan + "\nNormal Sequence Tick Count: " + console.yellow + "{}".format(normal_count) + console.reset)
    # Hitherto, the decorator modified the sequence permanently, so the tick count for the latter
    # would be the same. This behaviour is confusing - the decorators should always create
    # *new* classes so there is only one policy for decoration - instance modification only.
    assert(normal_count == 2)


def test_priority_interrupt():
    print(console.bold + "\n****************************************************************************************" + console.reset)
    print(console.bold + "* Priority Interrupt" + console.reset)
    print(console.bold + "****************************************************************************************\n" + console.reset)
    oneshot = py_trees.meta.oneshot(py_trees.composites.Sequence)(name="Oneshot",
                                                                  children=[py_trees.behaviours.Failure(name="Failure")])
    # Tree with higher priority branch, Higher
    root = py_trees.composites.Selector(name="Root")
    fail_after_one = py_trees.behaviours.Count(name="HighPriority", fail_until=1, running_until=1, success_until=2)
    root.add_children([fail_after_one, oneshot])
    tick_tree(root=root, oneshot=oneshot, from_tick=1, to_tick=2)
    print(console.cyan + "\nOneShot Status (After Interrupt): " + console.yellow + "{}".format(oneshot.status) + console.reset)
    assert(oneshot.status == py_trees.common.Status.INVALID)
    final_count = tick_tree(root=root, oneshot=oneshot, from_tick=3, to_tick=3)
    print(console.cyan + "\nTick Count (After Resumption): " + console.yellow + "{}".format(final_count) + console.reset)
    assert(final_count == 3)


def test_running_sequence():
    """
    Makes sure proper responses are returned from a oneshot sequence
    with running children.
    """
    print(console.bold + "\n****************************************************************************************" + console.reset)
    print(console.bold + "* Running Sequence" + console.reset)
    print(console.bold + "****************************************************************************************\n" + console.reset)
    run_a_bit = py_trees.behaviours.Count(name="RunABit", fail_until=0, running_until=2, success_until=5)
    oneshot = py_trees.meta.oneshot(py_trees.composites.Sequence)(
        name="Oneshot",
        children=[run_a_bit])
    count = tick_tree(root=oneshot, oneshot=oneshot, from_tick=1, to_tick=4)
    assert(count == 1)


def test_oneshot_does_not_re_initialise():
    """
    Makes sure that a oneshot behaviour does not re-initialise once it
    has run through to completion (SUCCESS || FAILURE). It also checks
    to make sure it blocks on needless termination calls coming from
    priority interrupts / resets.
    """
    print(console.bold + "\n****************************************************************************************" + console.reset)
    print(console.bold + "* OneShot does not ReIinitialise" + console.reset)
    print(console.bold + "****************************************************************************************\n" + console.reset)
    success = py_trees.behaviours.Success(name="Success")
    tester = py_trees.meta.oneshot(CounterTester)(
        name="Tester",
        fail_until=0,
        running_until=0,
        success_until=1000,
        reset=False)
    root = py_trees.composites.Sequence(
        name="Sequence", children=[success, tester])
    tick_tree(root, tester, 1, 5)
    assert(tester.status == py_trees.common.Status.SUCCESS)
    print("\n")
    print(console.cyan + "Terminate Count: " + console.yellow + "{}".format(tester.terminate_count) + console.reset)
    print(console.cyan + "Initialize Count: " + console.yellow + "{}".format(tester.initialise_count) + console.reset)
    assert(tester.initialise_count == 1)


def test_oneshot_does_re_initialise():
    print(console.bold + "\n****************************************************************************************" + console.reset)
    print(console.bold + "* OneShot does ReIinitialise" + console.reset)
    print(console.bold + "****************************************************************************************\n" + console.reset)
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
                              py_trees.common.Status.INVALID,
                              py_trees.common.Status.SUCCESS,
                              py_trees.common.Status.INVALID,
                              py_trees.common.Status.SUCCESS,
                              py_trees.common.Status.INVALID
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
        py_trees.display.print_ascii_tree(root, 0, show_status=True)
        assert(root.status == root_expected_status[i])
        assert(tester.status == tester_expected_status[i])
        assert(periodic_success.status == periodic_expected_status[i])
    print("\n")
    print(console.cyan + "Terminate Count: " + console.yellow + "{}".format(tester.terminate_count) + console.reset)
    print(console.cyan + "Initialize Count: " + console.yellow + "{}".format(tester.initialise_count) + console.reset)
    assert(tester.terminate_count == 3)
    assert(tester.initialise_count == 3)
