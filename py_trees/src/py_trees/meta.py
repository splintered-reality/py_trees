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

"""

##############################################################################
# Imports
##############################################################################

import functools
import time

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

            # aliases to original variables/methods
            self.blackbox_level = self.original.blackbox_level
            self.children = self.original.children
            self.setup = self.original.setup
            self.initialise = self.original.initialise
            self.terminate = self.original.terminate
            # id is important to match for composites...the children must relate to the correct parent id
            self.id = self.original.id

        def update(self):
            """
            This is the usual method where extra functionality
            is added.
            """
            self.original.tick_once()
            return self.original.status

        def __getattr__(self, name):
            """
            So we can pull extra attributes in the original above and beyond the behaviour attributes.
            """
            return getattr(self.original, name)

    return Imposter

# ##############################################################################
# # Timeout
# ##############################################################################


def timeout(cls, duration):
    """
    A decorator that applies a timeout pattern to an existing behaviour.

    .. code-block:: python

        work_with_timeout = py_trees.meta.Timeout(WorkBehaviour, 10.0)(name="Work")
    """

    def _timeout_init(func, duration):
        """
        Replace the default tick with one which runs the original function only if
        the oneshot variable is unset, yielding the unmodified object otherwise.
        """
        @functools.wraps(func)
        def wrapped(self, *args, **kwargs):
            func(self, *args, **kwargs)
            self.duration = duration
            self.finish_time = None
        return wrapped

    def _timeout_initialise(func):
        @functools.wraps(func)
        def wrapped(self, *args, **kwargs):
            if self.finish_time is None:
                self.finish_time = time.time() + self.duration
        return wrapped

    def _timeout_update(func):
        @functools.wraps(func)
        def wrapped(self, *args, **kwargs):
            self.original.tick_once()
            current_time = time.time()
            if current_time > self.finish_time:
                self.feedback_message = "timed out"
                return common.Status.FAILURE
            else:
                self.feedback_message = self.original.feedback_message + " [time left: %s]" % (self.finish_time - current_time)
                return self.original.status
        return wrapped

    def _timeout_terminate(func):
        @functools.wraps(func)
        def wrapped(self, new_status):
            if new_status != common.Status.RUNNING:
                self.finish_time = None
        return wrapped

    Timeout = imposter(cls)
    setattr(Timeout, "__init__", _timeout_init(Timeout.__init__, duration))
    setattr(Timeout, "initialise", _timeout_initialise(Timeout.initialise))
    setattr(Timeout, "update", _timeout_update(Timeout.update))
    setattr(Timeout, "terminate", _timeout_terminate(Timeout.terminate))
    return Timeout


##############################################################################
# Inverter
##############################################################################


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
    def _update(func):
        @functools.wraps(func)
        def wrapped(self):
            self.original.tick_once()
            if self.original.status == common.Status.SUCCESS:
                return common.Status.FAILURE
                self.feedback_message = "success -> failure"
            elif self.original.status == common.Status.FAILURE:
                self.feedback_message = "failure -> success"
                return common.Status.SUCCESS
            else:
                self.feedback_message = self.original.feedback_message
                return self.original.status
        return wrapped

    Inverter = imposter(cls)
    setattr(Inverter, "update", _update(Inverter.update))
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
    def _update(func):
        @functools.wraps(func)
        def wrapped(self):
            self.original.tick_once()
            if self.original.status == common.Status.RUNNING:
                self.feedback_message = "running is failure [%s]" % self.original.feedback_message
                return common.Status.FAILURE
            else:
                self.feedback_message = self.original.feedback_message
                return self.original.status
        return wrapped

    RunningIsFailure = imposter(cls)
    setattr(RunningIsFailure, "update", _update(RunningIsFailure.update))
    return RunningIsFailure

#############################
# RunningIsSuccess
#############################


def running_is_success(cls):
    """
    Don't hang around...

    .. code-block:: python

       @running_is_success
       class DontHangAround(Pontificating)
           pass

    or

    .. code-block:: python

       dont_hang_around = running_is_success(Pontificating)("Greek Philosopher")
    """
    def _update(func):
        @functools.wraps(func)
        def wrapped(self):
            self.original.tick_once()
            if self.original.status == common.Status.RUNNING:
                self.feedback_message = "running is success [%s]" % self.original.feedback_message
                return common.Status.SUCCESS
            else:
                self.feedback_message = self.original.feedback_message
                return self.original.status
        return wrapped

    RunningIsSuccess = imposter(cls)
    setattr(RunningIsSuccess, "update", _update(RunningIsSuccess.update))
    return RunningIsSuccess

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
    def _update(func):
        @functools.wraps(func)
        def wrapped(self):
            self.original.tick_once()
            if self.original.status == common.Status.FAILURE:
                self.feedback_message = "failure is success [%s]" % self.original.feedback_message
                return common.Status.SUCCESS
            else:
                self.feedback_message = self.original.feedback_message
                return self.original.status
        return wrapped

    FailureIsSuccess = imposter(cls)
    setattr(FailureIsSuccess, "update", _update(FailureIsSuccess.update))
    return FailureIsSuccess

#############################
# FailureIsRunning
#############################


def failure_is_running(cls):
    """
    Be positive, always succeed.

    .. code-block:: python

       @failure_is_success
       class MustGoOnRegardless(ActedLikeAGoon)
           pass

    or

    .. code-block:: python

       must_go_on_regardless = failure_is_running(ActedLikeAGoon("Goon"))
    """
    def _update(func):
        @functools.wraps(func)
        def wrapped(self):
            self.original.tick_once()
            if self.original.status == common.Status.FAILURE:
                self.feedback_message = "failure is running [%s]" % self.original.feedback_message
                return common.Status.RUNNING
            else:
                self.feedback_message = self.original.feedback_message
                return self.original.status
        return wrapped

    FailureIsRunning = imposter(cls)
    setattr(FailureIsRunning, "update", _update(FailureIsRunning.update))
    return FailureIsRunning

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
    def _update(func):
        @functools.wraps(func)
        def wrapped(self):
            self.original.tick_once()
            if self.original.status == common.Status.SUCCESS:
                self.feedback_message = "success is failure [%s]" % self.original.feedback_message
                return common.Status.FAILURE
            else:
                self.feedback_message = self.original.feedback_message
                return self.original.status
        return wrapped

    SuccessIsFailure = imposter(cls)
    setattr(SuccessIsFailure, "update", _update(SuccessIsFailure.update))
    return SuccessIsFailure

#############################
# SuccessIsRunning
#############################


def success_is_running(cls):
    """
    Be depressed, always fail.

    .. code-block:: python

       @success_is_running
       class TheEndIsSillNotNigh(ActingLikeAGoon)
           pass

    or

    .. code-block:: python

       the_end_is_still_not_night = success_is_running(ActingLikeAGoon("Goon"))
    """
    def _update(func):
        @functools.wraps(func)
        def wrapped(self):
            self.original.tick_once()
            if self.original.status == common.Status.SUCCESS:
                self.feedback_message = "success is running [%s]" % self.original.feedback_message
                return common.Status.RUNNING
            else:
                self.feedback_message = self.original.feedback_message
                return self.original.status
        return wrapped

    SuccessIsRunning = imposter(cls)
    setattr(SuccessIsRunning, "update", _update(SuccessIsRunning.update))
    return SuccessIsRunning
