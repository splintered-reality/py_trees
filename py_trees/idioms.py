#!/usr/bin/env python
#
# License: BSD
#   https://raw.githubusercontent.com/splintered-reality/py_trees/devel/LICENSE
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

from typing import List
import uuid

from . import behaviour
from . import behaviours
from . import blackboard
from . import common
from . import composites
from . import decorators

##############################################################################
# Creational Methods
##############################################################################


def pick_up_where_you_left_off(
        name="Pickup Where You Left Off Idiom",
        tasks=[]):
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
        task_guard = behaviours.CheckBlackboardVariableValue(
            name="Done?",
            variable_name=task.name.lower().replace(" ", "_") + "_done",
            expected_value=True
        )
        sequence = composites.Sequence(name="Worker")
        mark_task_done = behaviours.SetBlackboardVariable(
            name="Mark\n" + task.name.lower().replace(" ", "_") + "_done",
            variable_name=task.name.lower().replace(" ", "_") + "_done",
            variable_value=True
        )
        sequence.add_children([task, mark_task_done])
        task_selector.add_children([task_guard, sequence])
        root.add_child(task_selector)
    for task in tasks:
        clear_mark_done = behaviours.UnsetBlackboardVariable(
            name="Clear\n" + task.name.lower().replace(" ", "_") + "_done",
            key=task.name.lower().replace(" ", "_") + "_done"
        )
        root.add_child(clear_mark_done)
    return root


def eternal_guard(
        subtree: behaviour.Behaviour,
        name: str="Eternal Guard",
        conditions: List[behaviour.Behaviour]=[],
        blackboard_variable_prefix: str=None) -> behaviour.Behaviour:
    """
    The eternal guard idiom implements a stronger :term:`guard` than the typical check at the
    beginning of a sequence of tasks. Here they guard continuously while the task sequence
    is being executed. While executing, if any of the guards should update with
    status :data:`~common.Status.FAILURE`, then the task sequence is terminated.

    .. graphviz:: dot/idiom-eternal-guard.dot
        :align: center

    Args:
        subtree: behaviour(s) that actually do the work
        name: the name to use on the root behaviour of the idiom subtree
        conditions: behaviours on which tasks are conditional
        blackboard_variable_prefix: applied to condition variable results stored on the blackboard (default: derived from the idiom name)

    Returns:
        the root of the idiom subtree

    .. seealso:: :class:`py_trees.decorators.EternalGuard`
    """
    if blackboard_variable_prefix is None:
        blackboard_variable_prefix = name.lower().replace(" ", "_")
    blackboard_variable_names = []
    # construct simple, easy to read, variable names (risk of conflict)
    counter = 1
    for condition in conditions:
        suffix = "" if len(conditions) == 1 else "_{}".format(counter)
        blackboard_variable_names.append(
            blackboard.Blackboard.separator +
            blackboard_variable_prefix +
            "_condition" +
            suffix
        )
        counter += 1
    # if there is just one blackboard name already on the blackboard, switch to unique names
    conflict = False
    for name in blackboard_variable_names:
        try:
            unused_name = blackboard.Blackboard.get(name)
            conflict = True
        except KeyError:
            pass
    if conflict:
        blackboard_variable_names = []
        counter = 1
        unique_id = uuid.uuid4()
        for condition in conditions:
            suffix = "" if len(conditions) == 1 else "_{}".format(counter)
            blackboard_variable_names.append(blackboard_variable_prefix + "_" + str(unique_id) + "_condition" + suffix)
            counter += 1
    # build the tree
    root = composites.Parallel(
        name=name,
        policy=common.ParallelPolicy.SuccessOnAll(synchronise=False)
    )
    guarded_tasks = composites.Selector(name="Guarded Tasks")
    for condition, blackboard_variable_name in zip(conditions, blackboard_variable_names):
        decorated_condition = decorators.StatusToBlackboard(
            name="StatusToBB",
            child=condition,
            variable_name=blackboard_variable_name
        )
        root.add_child(decorated_condition)
        guarded_tasks.add_child(
            behaviours.CheckBlackboardVariableValue(
                name="Abort on\n{}".format(condition.name),
                variable_name=blackboard_variable_name,
                expected_value=common.Status.FAILURE
            )
        )
    guarded_tasks.add_child(subtree)
    root.add_child(guarded_tasks)
    return root


def oneshot(
        behaviour: behaviour.Behaviour,
        name: str="Oneshot",
        variable_name: str="oneshot",
        policy: common.OneShotPolicy=common.OneShotPolicy.ON_SUCCESSFUL_COMPLETION
) -> behaviour.Behaviour:
    """
    Ensure that a particular pattern is executed through to
    completion just once. Thereafter it will just rebound with the completion status.

    .. graphviz:: dot/oneshot.dot

    .. note::

       Set the policy to configure the oneshot to keep trying if failing, or to abort
       further attempts regardless of whether it finished with status
       :data:`~py_trees.common.Status.SUCCESS`||:data:`~py_trees.common.Status.FAILURE`.

    Args:
        behaviour: single behaviour or composited subtree to oneshot
        name: the name to use for the oneshot root (selector)
        variable_name: name for the variable used on the blackboard, may be nested
        policy: execute just once regardless of success or failure, or keep trying if failing

    Returns:
        :class:`~py_trees.behaviour.Behaviour`: the root of the oneshot subtree

    .. seealso:: :class:`py_trees.decorators.OneShot`
    """
    subtree_root = composites.Selector(name=name)
    oneshot_with_guard = composites.Sequence(
        name="Oneshot w/ Guard")
    check_not_done = decorators.Inverter(
        name="Not Completed?",
        child=behaviours.CheckBlackboardVariableExists(
            name="Completed?",
            variable_name=variable_name
        )
    )
    set_flag_on_success = behaviours.SetBlackboardVariable(
        name="Mark Done\n[SUCCESS]",
        variable_name=variable_name,
        variable_value=common.Status.SUCCESS
    )
    # If it's a sequence, don't double-nest it in a redundant manner
    if isinstance(behaviour, composites.Sequence):
        behaviour.add_child(set_flag_on_success)
        sequence = behaviour
    else:
        sequence = composites.Sequence(name="OneShot")
        sequence.add_children([behaviour, set_flag_on_success])

    oneshot_with_guard.add_child(check_not_done)
    if policy == common.OneShotPolicy.ON_SUCCESSFUL_COMPLETION:
        oneshot_with_guard.add_child(sequence)
    else:  # ON_COMPLETION (SUCCESS || FAILURE)
        oneshot_handler = composites.Selector(name="Oneshot Handler")
        bookkeeping = composites.Sequence(name="Bookkeeping")
        set_flag_on_failure = behaviours.SetBlackboardVariable(
            name="Mark Done\n[FAILURE]",
            variable_name=variable_name,
            variable_value=common.Status.FAILURE
        )
        bookkeeping.add_children(
            [set_flag_on_failure,
             behaviours.Failure(name="Failure")
             ])
        oneshot_handler.add_children([sequence, bookkeeping])
        oneshot_with_guard.add_child(oneshot_handler)

    oneshot_result = behaviours.CheckBlackboardVariableValue(
        name="Oneshot Result",
        variable_name=variable_name,
        expected_value=common.Status.SUCCESS,
    )
    subtree_root.add_children([oneshot_with_guard, oneshot_result])
    return subtree_root
