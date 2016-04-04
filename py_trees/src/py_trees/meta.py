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

import functools

from . import behaviour
from . import common

##############################################################################
# Utility Methods
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
    return type(class_name, (behaviour.Behaviour,), dict(update=func))


##############################################################################
# Some Machinery
##############################################################################

def imposter(cls):
    """
    A decorator that creates a generic behaviour impersonating
    (encapsulating) another behaviour. This gets used in turn
    by many of the other behaviour decorators, but is also useful in itself.

    Use it by replacing any of its internals, or inherit and overload from
    the created class

    .. code-block:: python

        def _update(self):
            self.original.tick_once()
            if self.original.status == common.Status.FAILURE:
                return common.Status.SUCCESS
            else:
                return self.original.status

        FailureIsSuccess = imposter(py_trees.behaviours.Failure)
        setattr(FailureIsSuccess, "update", _update)

    .. code-block:: python

        class FailureIsSuccess(imposter(py_trees.behaviours.Failure)):

            def __init__(self, *args, **kwargs):
                super(FailureIsSuccess, self).__init__(*args, **kwargs)

            def update(self):
                self.original.tick_once()
                if self.original.status == common.Status.FAILURE:
                    return common.Status.SUCCESS
                else:
                    return self.original.status
    """
    class Imposter(behaviour.Behaviour):
        def __init__(self, *args, **kwargs):
            """
            Pass on the arguments intact except for the name. That is
            prefixed with an underscore to denote that it is internal and
            not linked in the usual fashion.
            """
            if 'name' in kwargs:
                name = kwargs['name']
                new_args = args
            else:
                new_args = list(args)
                name = new_args[0]
                new_args[0] = "_" + name
                new_args = tuple(new_args)
            super(Imposter, self).__init__(name)
            self.original = cls(*new_args, **kwargs)

        def setup(self, timeout):
            """
            Call the underlying setup function.
            """
            self.original.setup(timeout)

        def initialise(self):
            """
            Call the underlying initialise function.
            """
            self.original.initialise()

        def update(self):
            """
            This is the usual method where extra functionality
            is added.
            """
            self.original.tick_once()
            return self.original.status
    return Imposter


#############################
# Inverter
#############################


def inverter(cls):
    """
    Inverts the result of a class's update function.

    .. code-block:: python

       @inverter
       class Failure(Success)
           pass

    or

    .. code-block:: python

       failure = inverter(Success)("Failure")

    """
    def _update(self):
        self.original.tick_once()
        if self.original.status == common.Status.SUCCESS:
            return common.Status.FAILURE
        elif self.original.status == common.Status.FAILURE:
            return common.Status.SUCCESS
        else:
            return self.original.status

    Inverter = imposter(cls)
    setattr(Inverter, "update", _update)
    return Inverter

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

    .. code-block:: python

       @oneshot
       class DoOrDie(GimmeASecondChance)
           pass

    or

    .. code-block:: python

       do_or_die = gimme_a_second_chance(GimmeASecondChance)("Do or Die")
    """
    setattr(cls, "tick", _oneshot_tick(cls.tick))
    return cls

#############################
# RunningIsFailure
#############################


def running_is_failure(cls):
    """
    Got to be snappy! We want results...yesterday!

    .. code-block:: python

       @running_is_failure
       class NeedResultsNow(Pontificating)
           pass

    or

    .. code-block:: python

       need_results_now = running_is_failure(Pontificating)("Greek Philosopher")
    """
    def _update(self):
        self.original.tick_once()
        if self.original.status == common.Status.RUNNING:
            return common.Status.FAILURE
        else:
            return self.original.status

    RunningIsFailure = imposter(cls)
    setattr(RunningIsFailure, "update", _update)
    return RunningIsFailure

#############################
# FailureIsSuccess
#############################


def failure_is_success(cls):
    """
    Be positive, always succeed.

    .. code-block:: python

       @failure_is_success
       class MustGoOnRegardless(ActingLikeAGoon)
           pass

    or

    .. code-block:: python

       must_go_on_regardless = failure_is_success(ActingLikeAGoon("Goon"))
    """

    def _update(self):
        self.original.tick_once()
        if self.original.status == common.Status.FAILURE:
            return common.Status.SUCCESS
        else:
            return self.original.status

    FailureIsSuccess = imposter(cls)
    setattr(FailureIsSuccess, "update", _update)
    return FailureIsSuccess

#############################
# SuccessIsFailure
#############################


def success_is_failure(cls):
    """
    Be depressed, always fail.

    .. code-block:: python

       @success_is_failure
       class TheEndIsNigh(ActingLikeAGoon)
           pass

    or

    .. code-block:: python

       the_end_is_night = success_is_failure(ActingLikeAGoon("Goon"))
    """
    def _update(self):
        self.original.tick_once()
        if self.original.status == common.Status.SUCCESS:
            return common.Status.FAILURE
        else:
            return self.original.status

    SuccessIsFailure = imposter(cls)
    setattr(SuccessIsFailure, "update", _update)
    return SuccessIsFailure
