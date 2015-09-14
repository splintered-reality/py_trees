#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/yujinrobot/gopher_crazy_hospital/license/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
.. module:: meta
   :platform: Unix
   :synopsis: Behaviour creation utilities.

Usually in the form of factory-like functions or decorators.

----

"""

##############################################################################
# Imports
##############################################################################

from . import common
from .behaviour import Behaviour

##############################################################################
# Behaviour Metaprogramming
##############################################################################


def create_behaviour_from_function(func):
    """
    Create a behaviour from the specified function, swapping it in place of
    the default update function.

    :param func: function to be used in place of the default :py:meth:`Behaviour.update() <py_trees.behaviours.Behaviour.update>` function.
    :type func: any function with just one argument for 'self', must return Status
    """
    class_name = func.__name__.capitalize()
    # globals()[class_name] = type(class_name, (Behaviour,), dict(update=func))
    return type(class_name, (Behaviour,), dict(update=func))

#############################
# Inverter
#############################


def _invert(func):
    """Inverts the function, used for the inverter decorator."""
    def wrapped(*args, **kwargs):
        status = func(*args, **kwargs)
        return common.Status.FAILURE if (status == common.Status.SUCCESS) else status
    return wrapped


def inverter(cls):
    """
    Inverts the result of a class's update function.

    .. code-block:: python

       @inverter
       class Failure(Success)
           pass

    or

    .. code-block:: python

       failure = inverter(Success("Failure"))
    """
    update = getattr(cls, "update")
    setattr(cls, "update", _invert(update))
    return cls


#############################
# FailureIsSuccess
#############################


def _failure_is_success(func):
    """Flips all failure to be success, used for the 'failure is success' decorator."""
    def wrapped(*args, **kwargs):
        status = func(*args, **kwargs)
        return common.Status.SUCCESS if (status == common.Status.FAILURE) else status
    return wrapped


def failure_is_success(cls):
    """
    Inverts the result of a class's update function.

    .. code-block:: python

       @failure_is_success
       class MustGoOnRegardless(ActingLikeAGoon)
           pass

    or

    .. code-block:: python

       must_go_on_regardless = failure_is_success(ActingLikeAGoon("Goon"))
    """
    update = getattr(cls, "update")
    setattr(cls, "update", _failure_is_success(update))
    return cls
