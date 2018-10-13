#!/usr/bin/env python
#
# License: BSD
#   https://raw.githubusercontent.com/stonier/py_trees/devel/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
A library of subtree creators that build complex patterns of behaviours
representing common behaviour tree idioms.
"""

##############################################################################
# Imports
##############################################################################

from . import blackboard
from . import common
from . import composites
# from . import meta

##############################################################################
# Creational Methods
##############################################################################

def pick_up_where_you_left_off(name, tasks):
    """
    Rudely interrupted while enjoying a sandwich, a caveman (just because
    they wore loincloths does not mean they were not civilised), picks
    up his club and fends off the sabre-tooth tiger invading his sanctum
    as if he were swatting away a gnat. Task accomplished, he returns
    to the joys of munching through the layers of his sandwich.

    .. graphviz:: dot/pick_up_where_you_left_off.dot

    .. note::

       There are alternative ways to accomplish this idiom with their
       pros and cons.

       a) The tasks in the sequence could be replaced by a
       factory behaviour that dynamically checks the state of play and
       spins up the tasks required each time the task sequence is first
       entered and invalidates/deletes them when it is either finished
       or invalidated. That has the advantage of not requiring much of
       the blackboard machinery here, but disadvantage in not making
       visible the task sequence itself at all times (i.e. burying
       details under the hood).

       b) A new composite which retains the index between
       initialisations can also achieve the same pattern with fewer
       blackboard shenanigans, but suffers from an increased
       logical complexity cost for your trees (each new composite
       increases decision making complexity (O(n!)).

    Args:
        name (:obj:`str`): the name to use for the task sequence behaviour
        tasks ([:class:`~py_trees.behaviour.Behaviour`): lists of tasks to be sequentially performed

    Returns:
        :class:`~py_trees.behaviour.Behaviour`: root of the generated subtree
    """
    root = composites.Sequence(name=name)
    for task in tasks:
        task_selector = composites.Selector(name="Do or Don't")
        task_guard = blackboard.CheckBlackboardVariable(
            name="Done?",
            variable_name=task.name.lower().replace(" ", "_") + "_done",
            expected_value=True,
            clearing_policy=common.ClearingPolicy.NEVER
        )
        sequence = composites.Sequence(name="Worker")
        mark_task_done = blackboard.SetBlackboardVariable(
            name="Mark\n" + task.name.lower().replace(" ", "_") + "_done",
            variable_name=task.name.lower().replace(" ", "_") + "_done",
            variable_value=True
        )
        sequence.add_children([task, mark_task_done])
        task_selector.add_children([task_guard, sequence])
        root.add_child(task_selector)
    for task in tasks:
        clear_mark_done = blackboard.ClearBlackboardVariable(
            name="Clear\n" + task.name.lower().replace(" ", "_") + "_done",
            variable_name=task.name.lower().replace(" ", "_") + "_done"
        )
        root.add_child(clear_mark_done)
    return root

def oneshot(name, variable_name, behaviour):
    """
    Ensure that a particular pattern is executed through to *successful*
    completion just once. Thereafter it will just rebound with success.

    .. graphviz:: dot/oneshot.dot

    .. note::

       This does not prevent re-entry when executed through to completion
       with result :data:`~py_trees.common.Status.FAILURE`.

    Args:
        name (:obj:`str`): the name to use for the oneshot root (selector)
        variable_name (:obj:`str`): name for the flag used on the blackboard (ensure it is unique)
        behaviour (:class:`~py_trees.behaviour.Behaviour`): beheaviour or root of a subtree to oneshot

    Returns:
        :class:`~py_trees.behaviour.Behaviour`: the root of the oneshot subtree
    """
    subtree_root = composites.Selector(name=name)
    check_flag = blackboard.CheckBlackboardVariable(
        name="Done?",
        variable_name=variable_name,
        expected_value=True,
        clearing_policy=common.ClearingPolicy.NEVER
    )
    set_flag = blackboard.SetBlackboardVariable(
        name="Mark Done",
        variable_name=variable_name,
        variable_value=True
    )
    # If it's a sequence, don't double-nest it in a redundant manner
    if isinstance(behaviour, composites.Sequence):
        behaviour.add_child(set_flag)
        sequence = behaviour
    else:
        sequence = composites.Sequence(name="OneShot")
        sequence.add_children([behaviour, set_flag])

    subtree_root.add_children([check_flag, sequence])
    return subtree_root
