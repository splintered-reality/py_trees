#!/usr/bin/env python
#
# License: BSD
#   https://raw.githubusercontent.com/splintered-reality/py_trees/devel/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""Creators of common subtree patterns."""

##############################################################################
# Imports
##############################################################################

import operator
import typing

from . import behaviour, behaviours, blackboard, common, composites, decorators

##############################################################################
# Creational Methods
##############################################################################


def pick_up_where_you_left_off(
    name: str, tasks: typing.List[behaviour.BehaviourSubClass]
) -> behaviour.Behaviour:
    """
    Create an idiom that enables a sequence of tasks to pick up where it left off.

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
    root = composites.Sequence(name=name, memory=True)
    for task in tasks:
        task_selector = composites.Selector(name="Do or Don't", memory=False)
        task_guard = behaviours.CheckBlackboardVariableValue(
            name="Done?",
            check=common.ComparisonExpression(
                variable=task.name.lower().replace(" ", "_") + "_done",
                value=True,
                operator=operator.eq,
            ),
        )
        sequence = composites.Sequence(name="Worker", memory=True)
        mark_task_done = behaviours.SetBlackboardVariable(
            name="Mark\n" + task.name.lower().replace(" ", "_") + "_done",
            variable_name=task.name.lower().replace(" ", "_") + "_done",
            variable_value=True,
            overwrite=True,
        )
        sequence.add_children([task, mark_task_done])
        task_selector.add_children([task_guard, sequence])
        root.add_child(task_selector)
    for task in tasks:
        clear_mark_done = behaviours.UnsetBlackboardVariable(
            name="Clear\n" + task.name.lower().replace(" ", "_") + "_done",
            key=task.name.lower().replace(" ", "_") + "_done",
        )
        root.add_child(clear_mark_done)
    return root


def either_or(
    conditions: typing.List[common.ComparisonExpression],
    subtrees: typing.List[behaviour.Behaviour],
    name: str = "Either Or",
    namespace: typing.Optional[str] = None,
) -> behaviour.Behaviour:
    """
    Create an idiom with selector-like qualities, but no priority concerns.

    Often you need a kind of selector that doesn't implement prioritisations, i.e.
    you would like different paths to be selected on a first-come, first-served basis.

    .. code-block:: python

        task_one = py_trees.behaviours.TickCounter(name="Subtree 1", duration=2)
        task_two = py_trees.behaviours.TickCounter(name="Subtree 2", duration=2)
        either_or = py_trees.idioms.either_or(
            name="EitherOr",
            conditions=[
                py_trees.common.ComparisonExpression("joystick_one", "enabled", operator.eq),
                py_trees.common.ComparisonExpression("joystick_two", "enabled", operator.eq),
            ],
            subtrees=[task_one, task_two],
            namespace="either_or",
        )

    .. graphviz:: dot/idiom-either-or.dot
        :align: center
        :caption: Idiom - Either Or

    Up front is an XOR conditional check which locks in the result on the blackboard
    under the specified namespace. Locking the result in permits the conditional
    variables to vary in future ticks without interrupting the execution of the
    chosen subtree (an example of a conditional variable may be one that has
    registered joystick button presses).

    Once the result is locked in, the relevant subtree is activated beneath the
    selector. The children of the selector are, from left to right, not in any
    order of priority since the previous xor choice has been locked in and isn't
    revisited until the subtree executes to completion. Only one
    may be active and it cannot be interrupted by the others.

    The only means of interrupting the execution is via a higher priority in the
    tree that this idiom is embedded in.

    Args:
        conditions: list of triggers that ultimately select the subtree to enable
        subtrees: list of subtrees to tick from in the either_or operation
        name: the name to use for this idiom's root behaviour
        preemptible: whether the subtrees may preempt (interrupt) each other
        namespace: this idiom's private variables will be put behind this namespace

    Raises:
        ValueError if the number of conditions does not match the number of subtrees

    If no namespace is provided, a unique one is derived from the idiom's name.

    .. seealso:: :ref:`py-trees-demo-either-or <py-trees-demo-either-or-program>`

    .. todo:: a version for which other subtrees can preempt (in an unprioritised manner) the active branch
    """
    if len(conditions) != len(subtrees):
        raise ValueError(
            "Must be the same number of conditions as subtrees [{} != {}]".format(
                len(conditions), len(subtrees)
            )
        )
    root = composites.Sequence(name=name, memory=True)
    configured_namespace: str = (
        namespace
        if namespace is not None
        else blackboard.Blackboard.separator
        + name.lower().replace("-", "_").replace(" ", "_")
        + blackboard.Blackboard.separator
        + str(root.id).replace("-", "_").replace(" ", "_")
        + blackboard.Blackboard.separator
        + "conditions"
    )
    xor = behaviours.CheckBlackboardVariableValues(
        name="XOR",
        checks=conditions,
        operator=operator.xor,
        namespace=configured_namespace,
    )
    chooser = composites.Selector(name="Chooser", memory=False)
    for counter in range(1, len(conditions) + 1):
        sequence = composites.Sequence(
            name="Option {}".format(str(counter)), memory=True
        )
        variable_name = (
            configured_namespace + blackboard.Blackboard.separator + str(counter)
        )
        disabled = behaviours.CheckBlackboardVariableValue(
            name="Enabled?",
            check=common.ComparisonExpression(
                variable=variable_name, value=True, operator=operator.eq
            ),
        )
        sequence.add_children([disabled, subtrees[counter - 1]])
        chooser.add_child(sequence)
    root.add_children([xor, chooser])
    return root


def oneshot(
    behaviour: behaviour.Behaviour,
    name: str = "Oneshot",
    variable_name: str = "oneshot",
    policy: common.OneShotPolicy = common.OneShotPolicy.ON_SUCCESSFUL_COMPLETION,
) -> behaviour.Behaviour:
    """
    Ensure that a particular pattern is executed through to completion just once.

    Thereafter it will just rebound with the completion status.

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
    subtree_root = composites.Selector(name=name, memory=False)
    oneshot_with_guard = composites.Sequence(name="Oneshot w/ Guard", memory=True)
    check_not_done = decorators.Inverter(
        name="Not Completed?",
        child=behaviours.CheckBlackboardVariableExists(
            name="Completed?", variable_name=variable_name
        ),
    )
    set_flag_on_success = behaviours.SetBlackboardVariable(
        name="Mark Done\n[SUCCESS]",
        variable_name=variable_name,
        variable_value=common.Status.SUCCESS,
        overwrite=True,
    )
    # If it's a sequence, don't double-nest it in a redundant manner
    if isinstance(behaviour, composites.Sequence):
        behaviour.add_child(set_flag_on_success)
        sequence = behaviour
    else:
        sequence = composites.Sequence(name="OneShot", memory=True)
        sequence.add_children([behaviour, set_flag_on_success])

    oneshot_with_guard.add_child(check_not_done)
    if policy == common.OneShotPolicy.ON_SUCCESSFUL_COMPLETION:
        oneshot_with_guard.add_child(sequence)
    else:  # ON_COMPLETION (SUCCESS || FAILURE)
        oneshot_handler = composites.Selector(name="Oneshot Handler", memory=False)
        bookkeeping = composites.Sequence(name="Bookkeeping", memory=True)
        set_flag_on_failure = behaviours.SetBlackboardVariable(
            name="Mark Done\n[FAILURE]",
            variable_name=variable_name,
            variable_value=common.Status.FAILURE,
            overwrite=True,
        )
        bookkeeping.add_children(
            [set_flag_on_failure, behaviours.Failure(name="Failure")]
        )
        oneshot_handler.add_children([sequence, bookkeeping])
        oneshot_with_guard.add_child(oneshot_handler)

    oneshot_result = behaviours.CheckBlackboardVariableValue(
        name="Oneshot Result",
        check=common.ComparisonExpression(
            variable=variable_name, value=common.Status.SUCCESS, operator=operator.eq
        ),
    )
    subtree_root.add_children([oneshot_with_guard, oneshot_result])
    return subtree_root
