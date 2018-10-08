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
from . import composites
# from . import meta

##############################################################################
# Creational Methods
##############################################################################

def create_pick_up_where_you_left_off_subtree():
    pass

def create_oneshot_subtree(name, variable_name, behaviour):
    """
    """
    # TODO smart check to see if it's a list of behavoiurs or just a single
    # behaviour.
    subtree_root = composites.Selector(name=name)
    check_flag = blackboard.CheckBlackboardVariable(
        name="Done?",
        variable_name=variable_name,
        expected_value=True,
        clearing_policy=py_trees.common.ClearingPolicy.NEVER
    )
    set_flag = blackboard.SetBlackboardVariable(
        name="Mark Done",
        variable_name=variable_name,
        variable_value=True
    )

    sequence = composites.Sequence(name="OneShot")
    sequence.add_children([behaviour, set_flag])

    subtree_root.add_children([check_flag, sequence])
    return subtree_root
