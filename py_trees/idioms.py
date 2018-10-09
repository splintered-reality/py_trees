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

from .common import Status
from .behaviour import Behaviour

from . import blackboard
from . import common
from . import composites
# from . import meta

##############################################################################
# Creational Methods
##############################################################################

def pick_up_where_you_left_off():
    pass

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
