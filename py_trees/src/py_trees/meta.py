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
import functools
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
        if status == common.Status.SUCCESS:
            return common.Status.FAILURE
        elif status == common.Status.FAILURE:
            return common.Status.SUCCESS
        else:
            return status
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

##############################################################################
# Oneshot
##############################################################################


def _oneshot_tick(func):
    """
    Replace the default tick with one which runs the original function only if
    the oneshot variable is unset, yielding the unmodified object otherwise.
    """
    @functools.wraps(func)
    def wrapped(self, *args, **kwargs):
        if self.status == common.Status.FAILURE or self.status == common.Status.SUCCESS:
            # if returned success/fail at any point, don't update or re-init
            yield self
        else:
            # otherwise, run the tick as normal yield from in python 3.3
            for child in func(self, *args, **kwargs):
                yield child

    return wrapped


def oneshot(cls):
    """
    Makes the given behaviour run only until it returns success or failure,
    retaining that state for all subsequent updates.
    """
    setattr(cls, "tick", _oneshot_tick(cls.tick))
    return cls

#############################
# RunningIsFailure
#############################


def _running_is_failure(func):
    def wrapped(*args, **kwargs):
        status = func(*args, **kwargs)
        return common.Status.FAILURE if (status == common.Status.RUNNING) else status
    return wrapped


def running_is_failure(cls):
    """
    Got to be snappy! We want results...yesterday!

    .. code-block:: python

       @running_is_failure
       class NeedResultsNow(Pontificating)
           pass

    or

    .. code-block:: python

       need_results_now = running_is_failure(Pontificating("Greek Philosopher"))
    """
    update = getattr(cls, "update")
    setattr(cls, "update", _running_is_failure(update))
    return cls

#############################
# FailureIsSuccess
#############################


def _failure_is_success(func):
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
