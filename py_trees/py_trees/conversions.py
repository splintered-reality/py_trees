#!/usr/bin/env python
#
# License: BSD
#   https://raw.githubusercontent.com/stonier/py_trees_suite/devel/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
.. module:: conversions
   :platform: Unix
   :synopsis: Functions used for converting between behaviour messages and internal representation

"""

##############################################################################
# Imports
##############################################################################

from . import composites
from . import common
from .behaviours import Behaviour
import unique_id
import uuid_msgs.msg as uuid_msgs
import py_trees_msgs.msg as py_trees_msgs

##############################################################################
# <etjpds
##############################################################################


def convert_type(behaviour):
    # problems with decorators?
    if isinstance(behaviour, composites.Sequence):
        return py_trees_msgs.Behaviour.SEQUENCE
    elif isinstance(behaviour, composites.Chooser):
        return py_trees_msgs.Behaviour.CHOOSER
    elif isinstance(behaviour, composites.Selector):
        return py_trees_msgs.Behaviour.SELECTOR
    elif isinstance(behaviour, composites.Parallel):
        return py_trees_msgs.Behaviour.PARALLEL
    elif isinstance(behaviour, Behaviour):
        return py_trees_msgs.Behaviour.BEHAVIOUR
    else:
        return 0  # unknown type


def convert_status(status):
    if status == common.Status.INVALID:
        return py_trees_msgs.Behaviour.INVALID
    elif status == common.Status.RUNNING:
        return py_trees_msgs.Behaviour.RUNNING
    elif status == common.Status.SUCCESS:
        return py_trees_msgs.Behaviour.SUCCESS
    elif status == common.Status.FAILURE:
        return py_trees_msgs.Behaviour.FAILURE
    else:
        return 0  # unknown status


def convert_blackbox_level(blackbox_level):
    if blackbox_level == common.BlackBoxLevel.DETAIL:
        return py_trees_msgs.Behaviour.BLACKBOX_LEVEL_DETAIL
    elif blackbox_level == common.BlackBoxLevel.COMPONENT:
        return py_trees_msgs.Behaviour.BLACKBOX_LEVEL_COMPONENT
    elif blackbox_level == common.BlackBoxLevel.BIG_PICTURE:
        return py_trees_msgs.Behaviour.BLACKBOX_LEVEL_BIG_PICTURE
    else:
        return py_trees_msgs.Behaviour.BLACKBOX_LEVEL_NOT_A_BLACKBOX


def behaviour_to_msg(behaviour):
    new_behaviour = py_trees_msgs.Behaviour()
    new_behaviour.name = behaviour.name
    new_behaviour.class_name = str(behaviour.__module__) + '.' + str(type(behaviour).__name__)
    new_behaviour.own_id = unique_id.toMsg(behaviour.id)
    new_behaviour.parent_id = unique_id.toMsg(behaviour.parent.id) if behaviour.parent else uuid_msgs.UniqueID()
    new_behaviour.child_ids = [unique_id.toMsg(child.id) for child in behaviour.iterate(direct_descendants=True) if not child.id == behaviour.id]

    tip = behaviour.tip()
    # tip_id is empty if the behaviour is invalid or if it is a valid
    # leaf
    if tip is not None and tip != behaviour:
        new_behaviour.tip_id = unique_id.toMsg(tip.id)

    new_behaviour.type = convert_type(behaviour)
    new_behaviour.blackbox_level = convert_blackbox_level(behaviour.blackbox_level)
    new_behaviour.status = convert_status(behaviour.status)
    new_behaviour.message = behaviour.feedback_message

    return new_behaviour
