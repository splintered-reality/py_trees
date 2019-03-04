#!/usr/bin/env python
#
# License: BSD
#   https://raw.githubusercontent.com/splintered-reality/py_trees/devel/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
Meta methods to create behaviours without needing to create the behaviours
themselves.
"""

##############################################################################
# Imports
##############################################################################

from . import behaviour
from . import common

##############################################################################
# Utility Methods
##############################################################################


def create_behaviour_from_function(func):
    """
    Create a behaviour from the specified function, dropping it in for
    the Behaviour :meth:`~py_trees.behaviour.Behaviour.update` method.
    Ths function must include the `self`
    argument and return a :class:`~py_trees.behaviours.common.Status` value.
    It also automatically provides a drop-in for the :meth:`~py_trees.behaviour.Behaviour.terminate`
    method that clears the feedback message. Other methods are left untouched.

    Args:
        func (:obj:`function`):  a drop-in for the :meth:`~py_trees.behaviour.Behaviour.update` method
    """
    class_name = func.__name__.capitalize()

    def init(self, name=class_name):
        behaviour.Behaviour.__init__(self, name=name)

    def terminate(self, new_status):
        if new_status == common.Status.INVALID:
            self.feedback_message = ""

    return type(class_name, (behaviour.Behaviour,), dict(__init__=init, update=func, terminate=terminate))
