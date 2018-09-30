#!/usr/bin/env python
#
# License: BSD
#   https://raw.githubusercontent.com/stonier/py_trees/devel/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
.. attention::
    This module is the least likely to remain stable in this package. It has
    only received cursory attention so far and a more thoughtful design for handling
    behaviour 'hats' might be needful at some point in the future.

Meta behaviours are created by utilising various programming techniques
pulled from a magic bag of tricks. Some of these minimise the effort to generate
a new behaviour while others provide mechanisms that greatly expand your
library of usable behaviours without having to increase the number of explicit
behaviours contained therein. The latter is achieved by providing a means for
behaviours to wear different 'hats' via python decorators.

.. image:: images/many-hats.png
   :width: 40px
   :align: center

Each function or decorator listed below includes its own example code
demonstrating its use.

**Factories**

* :func:`py_trees.meta.create_behaviour_from_function`
* :func:`py_trees.meta.create_imposter`

**Decorators (Hats)**

* :func:`py_trees.meta.condition`
* :func:`py_trees.meta.inverter`
* :func:`py_trees.meta.failure_is_running`
* :func:`py_trees.meta.failure_is_success`
* :func:`py_trees.meta.oneshot`
* :func:`py_trees.meta.running_is_failure`
* :func:`py_trees.meta.running_is_success`
* :func:`py_trees.meta.success_is_failure`
* :func:`py_trees.meta.success_is_running`
* :func:`py_trees.meta.timeout`
"""

##############################################################################
# Imports
##############################################################################

import functools
import time

from . import behaviour
from . import common
from . import composites

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

    def terminate(self, new_status):
        if new_status == common.Status.INVALID:
            self.feedback_message = ""

    # globals()[class_name] = type(class_name, (Behaviour,), dict(update=func))
    return type(class_name, (behaviour.Behaviour,), dict(update=func, terminate=terminate))


##############################################################################
# Some Machinery
##############################################################################

def create_imposter(cls):
    """
    Creates a new behaviour type impersonating (encapsulating) another
    behaviour type.

    This is primarily used to develop
    other decorators but can also be useful in itself. It takes care of
    the handles responsible for making the encapsulation work and
    leaves you with just the task of replacing the relevant modifications
    (usually to the :meth:`~py_trees.behaviour.Behaviour.update` method).
    The modifications can be made by direct replacement of methods or
    by inheriting and overriding them. See the examples below.

    Args:
        cls (:class:`~py_trees.behaviour.Behaviour`): an existing behaviour class type

    Returns:
        :class:`~py_trees.behaviour.Behaviour`: the new encapsulated behaviour class

    Examples:

        Replacing methods:

        .. code-block:: python

            def _update(self):
                self.original.tick_once()
                if self.original.status == common.Status.FAILURE:
                    return common.Status.SUCCESS
                else:
                    return self.original.status

            FailureIsSuccess = create_imposter(py_trees.behaviours.Failure)
            setattr(FailureIsSuccess, "update", _update)

        Subclassing and overriding:

        .. code-block:: python

            class FailureIsSuccess(create_imposter(py_trees.behaviours.Failure)):

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
                kwargs['name'] = str(kwargs['name'])
                name = kwargs['name']
                kwargs['name'] = "_" + kwargs['name']
            else:
                # if name is missing, give it a placeholder name to indicate it is internal
                name = "_Imposter"
                kwargs['name'] = name
            super(Imposter, self).__init__(name)
            self.original = cls(*args, **kwargs)

            # aliases to original variables/methods
            self.blackbox_level = self.original.blackbox_level
            self.children = self.original.children
            self.setup = self.original.setup
            # id is important to match for composites...the children must relate to the correct parent id
            self.id = self.original.id

            if isinstance(self.original, composites.Composite):
                # monkeypatch add_child
                def add_child(child):
                    assert isinstance(child, behaviour.Behaviour), "children must be behaviours, but you passed in %s" % type(child)
                    self.children.append(child)
                    child.parent = self
                    return child.id
                self.original.add_child = add_child

        def tip(self):
            """
            This function overrides :meth:`~py_trees.behaviour.Behaviour.tip`
            and provides a fused capability depending on whether the original
            behaviour is a composite or not. If it is composite, it relies on
            the composite's return value, else it uses it's own.

            Important not to use the original itself since it doesn't officially
            exist in a tree.
            """
            if isinstance(self.original, composites.Composite):
                return self.original.tip()
            else:
                return super(Imposter, self).tip()

        def tick(self):
            """
            This function overrides Behaviour.tick() and work the same way
            except it would not call initialise and stop methods on original
            and let the original's update handle it's state.

            There is some analysis explaining the need for this override in

                https://github.com/stonier/py_trees/issues/32

            :return py_trees.Behaviour: a reference to itself
            """
            self.logger.debug("%s.tick()" % (self.__class__.__name__))

            # this only initialises the imposter
            if self.status != common.Status.RUNNING:
                self.initialise()

            # initialise() and terminate() for the original behaviour
            # will be called from inside the original's tick()
            for sub_behaviour in self.original.tick():
                if sub_behaviour != self.original:
                    yield sub_behaviour
            new_status = self.update()
            if new_status not in list(common.Status):
                self.logger.error("A behaviour returned an invalid status, setting to INVALID [%s][%s]" % (new_status, self.name))
                new_status = common.Status.INVALID
            self.status = new_status
            yield self

        def update(self):
            """
            This is the usual method that gets replaced by
            the meta classes.
            """
            return self.original.status

        def terminate(self, new_status):
            """
            Imposter's custom implementation of termination that is called when
            higher priority interrupts occur. Note that stop/terminate are not called
            in the usual sequence of a tick, since that would double up on stop
            calls to the underlying original.
            """
            self.logger.debug("%s.terminate()[%s]" % (self.__class__.__name__, new_status))
            self.original.stop(new_status)
            self.status = self.original.status

        def __getattr__(self, name):
            """
            So we can pull extra attributes in the original above and beyond the behaviour attributes.
            """
            return getattr(self.original, name)

    return Imposter

##############################################################################
# Timeout
##############################################################################


def timeout(cls, duration):
    """
    A decorator that applies a timeout pattern to an existing behaviour.
    If the timeout is reached, the encapsulated behaviour's :meth:`~py_trees.behaviour.Behaviour.stop`
    method is called with status :data:`~py_trees.common.Status.FAILURE` otherwise it will
    simply directly tick and return with the same status
    as that of it's encapsulated behaviour.

    Args:
        cls (:class:`~py_trees.behaviour.Behaviour`): an existing behaviour class type
        duration (:obj:`float`): timeout length in seconds

    Returns:
        :class:`~py_trees.behaviour.Behaviour`: the modified behaviour class with timeout

    Examples:

        .. code-block:: python

           @py_trees.meta.timeout(10)
           class WorkBehaviour(py_trees.behaviour.Behaviour)

        or

        .. code-block:: python

            work_with_timeout = py_trees.meta.timeout(WorkBehaviour, 10.0)(name="Work")
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

    def _timeout_initialise(self):
        if self.finish_time is None:
            self.finish_time = time.time() + self.duration

    def _timeout_update(func):
        @functools.wraps(func)
        def wrapped(self, *args, **kwargs):
            # make sure this class initialises time related functions
            # when the underlying original will initialise itself
            if self.original.status != common.Status.RUNNING:
                self.initialise()
            current_time = time.time()
            if current_time > self.finish_time:
                self.feedback_message = "timed out"
                # invalidate the original (i.e. cancel it)
                self.original.stop(common.Status.INVALID)
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

    Timeout = create_imposter(cls)
    setattr(Timeout, "__init__", _timeout_init(Timeout.__init__, duration))
    setattr(Timeout, "initialise", _timeout_initialise)
    setattr(Timeout, "update", _timeout_update(Timeout.update))
    setattr(Timeout, "terminate", _timeout_terminate(Timeout.terminate))
    return Timeout

##############################################################################
# Oneshot
##############################################################################


def oneshot(cls):
    def _oneshot_init(func):
        @functools.wraps(func)
        def wrapped(self, *args, **kwargs):
            func(self, *args, **kwargs)
            self.final_status = None
        return wrapped

    def _oneshot_update(func):
        @functools.wraps(func)
        def wrapped(self, *args, **kwargs):
            self.logger.debug("OneShot.wrapped_update()")
            if self.final_status:
                return self.final_status
            else:
                if self.original.status in (common.Status.FAILURE, common.Status.SUCCESS):
                    self.final_status = self.original.status
                return self.original.status
        return wrapped

    def _oneshot_tick(func):
        @functools.wraps(func)
        def wrapped(self, *args, **kwargs):
            if self.final_status:
                self.status = self.final_status
                self.logger.debug("OneShot.wrapped_tick()[rebounding]")
                yield self
            else:
                self.logger.debug("OneShot.wrapped_tick()")
                for b in func(self):
                    yield b  # b = behaviour
        return wrapped

    def _oneshot_terminate(func):
        @functools.wraps(func)
        def wrapped(self, new_status):
            self.logger.debug("OneShot.wrapped_terminate()[{}]".format(new_status))
            # handle only the interrupt/reset case
            if new_status == common.Status.INVALID:
                if self.final_status:
                    self.status = new_status
                else:
                    self.original.stop(new_status)
                    self.status = self.original.status
        return wrapped

    OneShot = create_imposter(cls)
    setattr(OneShot, "__init__", _oneshot_init(OneShot.__init__))
    setattr(OneShot, "update", _oneshot_update(OneShot.update))
    setattr(OneShot, "tick", _oneshot_tick(OneShot.tick))
    setattr(OneShot, "terminate", _oneshot_terminate(OneShot.terminate))
    return OneShot

##############################################################################
# Inverter
##############################################################################


def inverter(cls):
    """
    A decorator that inverts the result of a class's update function.

    Args:
        cls (:class:`~py_trees.behaviour.Behaviour`): an existing behaviour class type

    Returns:
        :class:`~py_trees.behaviour.Behaviour`: the modified behaviour class

    Examples:
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
            if self.original.status == common.Status.SUCCESS:
                self.feedback_message = "success -> failure [{}]".format(self.original.feedback_message)
                return common.Status.FAILURE
            elif self.original.status == common.Status.FAILURE:
                self.feedback_message = "failure -> success [{}]".format(self.original.feedback_message)
                return common.Status.SUCCESS
            else:
                self.feedback_message = self.original.feedback_message
                return self.original.status
        return wrapped

    Inverter = create_imposter(cls)
    setattr(Inverter, "update", _update(Inverter.update))
    return Inverter

#############################
# RunningIsFailure
#############################


def running_is_failure(cls):
    """
    Got to be snappy! We want results...yesterday!

    Args:
        cls (:class:`~py_trees.behaviour.Behaviour`): an existing behaviour class type

    Returns:
        :class:`~py_trees.behaviour.Behaviour`: the modified behaviour class

    Examples:
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
            if self.original.status == common.Status.RUNNING:
                self.feedback_message = "running is failure" + (" [%s]" % self.original.feedback_message if self.original.feedback_message else "")
                return common.Status.FAILURE
            else:
                self.feedback_message = self.original.feedback_message
                return self.original.status
        return wrapped

    RunningIsFailure = create_imposter(cls)
    setattr(RunningIsFailure, "__name__", running_is_failure.__name__)
    setattr(RunningIsFailure, "update", _update(RunningIsFailure.update))
    return RunningIsFailure

#############################
# RunningIsSuccess
#############################


def running_is_success(cls):
    """
    Don't hang around...

    Args:
        cls (:class:`~py_trees.behaviour.Behaviour`): an existing behaviour class type

    Returns:
        :class:`~py_trees.behaviour.Behaviour`: the modified behaviour class

    Examples:
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
            if self.original.status == common.Status.RUNNING:
                self.feedback_message = "running is success" + (" [%s]" % self.original.feedback_message if self.original.feedback_message else "")
                return common.Status.SUCCESS
            else:
                self.feedback_message = self.original.feedback_message
                return self.original.status
        return wrapped

    RunningIsSuccess = create_imposter(cls)
    setattr(RunningIsSuccess, "__name__", running_is_success.__name__)
    setattr(RunningIsSuccess, "update", _update(RunningIsSuccess.update))
    return RunningIsSuccess

#############################
# FailureIsSuccess
#############################


def failure_is_success(cls):
    """
    Be positive, always succeed.

    Args:
        cls (:class:`~py_trees.behaviour.Behaviour`): an existing behaviour class type

    Returns:
        :class:`~py_trees.behaviour.Behaviour`: the modified behaviour class

    Examples:
        .. code-block:: python

           @failure_is_success
           class MustGoOnRegardless(ActedLikeAGoon)
               pass

        or

        .. code-block:: python

           must_go_on_regardless = failure_is_success(ActedLikeAGoon)(name="Goon")
    """
    def _update(func):
        @functools.wraps(func)
        def wrapped(self):
            if self.original.status == common.Status.FAILURE:
                self.feedback_message = "failure is success" + (" [%s]" % self.original.feedback_message if self.original.feedback_message else "")
                return common.Status.SUCCESS
            else:
                self.feedback_message = self.original.feedback_message
                return self.original.status
        return wrapped

    FailureIsSuccess = create_imposter(cls)
    setattr(FailureIsSuccess, "__name__", failure_is_success.__name__)
    setattr(FailureIsSuccess, "update", _update(FailureIsSuccess.update))
    return FailureIsSuccess

#############################
# FailureIsRunning
#############################


def failure_is_running(cls):
    """
    Dont stop running.

    Args:
        cls (:class:`~py_trees.behaviour.Behaviour`): an existing behaviour class type

    Returns:
        :class:`~py_trees.behaviour.Behaviour`: the modified behaviour class

    Examples:
        .. code-block:: python

           @failure_is_running
           class MustGoOnRegardless(ActingLikeAGoon)
               pass

        or

        .. code-block:: python

           must_go_on_regardless = failure_is_running(ActingLikeAGoon)(name="Goon")
    """
    def _update(func):
        @functools.wraps(func)
        def wrapped(self):
            if self.original.status == common.Status.FAILURE:
                self.feedback_message = "failure is running" + (" [%s]" % self.original.feedback_message if self.original.feedback_message else "")
                return common.Status.RUNNING
            else:
                self.feedback_message = self.original.feedback_message
                return self.original.status
        return wrapped

    FailureIsRunning = create_imposter(cls)
    setattr(FailureIsRunning, "__name__", failure_is_running.__name__)
    setattr(FailureIsRunning, "update", _update(FailureIsRunning.update))
    return FailureIsRunning

#############################
# SuccessIsFailure
#############################


def success_is_failure(cls):
    """
    Be depressed, always fail.

    Args:
        cls (:class:`~py_trees.behaviour.Behaviour`): an existing behaviour class type

    Returns:
        :class:`~py_trees.behaviour.Behaviour`: the modified behaviour class

    Examples:
        .. code-block:: python

           @success_is_failure
           class TheEndIsNigh(ActingLikeAGoon)
               pass

        or

        .. code-block:: python

           the_end_is_nigh = success_is_failure(ActingLikeAGoon)(name="Goon")
    """
    def _update(func):
        @functools.wraps(func)
        def wrapped(self):
            if self.original.status == common.Status.SUCCESS:
                self.feedback_message = "success is failure" + (" [%s]" % self.original.feedback_message if self.original.feedback_message else "")
                return common.Status.FAILURE
            else:
                self.feedback_message = self.original.feedback_message
                return self.original.status
        return wrapped

    SuccessIsFailure = create_imposter(cls)
    setattr(SuccessIsFailure, "__name__", success_is_failure.__name__)
    setattr(SuccessIsFailure, "update", _update(SuccessIsFailure.update))
    return SuccessIsFailure

#############################
# SuccessIsRunning
#############################


def success_is_running(cls):
    """
    It never ends...

    Args:
        cls (:class:`~py_trees.behaviour.Behaviour`): an existing behaviour class type

    Returns:
        :class:`~py_trees.behaviour.Behaviour`: the modified behaviour class

    Examples:
        .. code-block:: python

           @success_is_running
           class TheEndIsSillNotNigh(ActingLikeAGoon)
               pass

        or

        .. code-block:: python

           the_end_is_still_not_nigh = success_is_running(ActingLikeAGoon)(name="Goon")
    """
    def _update(func):
        @functools.wraps(func)
        def wrapped(self):
            if self.original.status == common.Status.SUCCESS:
                self.feedback_message = "success is running [%s]" % self.original.feedback_message
                return common.Status.RUNNING
            else:
                self.feedback_message = self.original.feedback_message
                return self.original.status
        return wrapped

    SuccessIsRunning = create_imposter(cls)
    setattr(SuccessIsRunning, "__name__", success_is_running.__name__)
    setattr(SuccessIsRunning, "update", _update(SuccessIsRunning.update))
    return SuccessIsRunning

#############################
# Condition
#############################


def condition(cls, status):
    """
    Encapsulates a behaviour and wait for it's status to flip to the
    desired state. This behaviour will tick with
    :data:`~py_trees.common.Status.RUNNING` while waiting and
    :data:`~py_trees.common.Status.SUCCESS` when the flip occurs.

    Args:
        cls (:class:`~py_trees.behaviour.Behaviour`): an existing behaviour class type
        status (:class:`~py_trees.common.Status`): the desired status to watch for

    Returns:
        :class:`~py_trees.behaviour.Behaviour`: the modified behaviour class

    Examples:
        .. code-block:: python

           @condition(py_trees.common.Status.RUNNING)
           class HangingAbout(WillStartSoon)
               pass

        or

        .. code-block:: python

           hanging_about = condition(WillStartSoon, py_trees.common.Status.RUNNING)(name="Hanging About")
    """
    def _init(func, status):
        """
        Replace the default init with one which also accepts the desired status
        to watch for.
        """
        @functools.wraps(func)
        def wrapped(self, *args, **kwargs):
            func(self, *args, **kwargs)
            self.succeed_status = status
        return wrapped

    def _update(func):
        @functools.wraps(func)
        def wrapped(self):
            self.logger.debug("%s.update()" % self.__class__.__name__)
            self.feedback_message = "'{0}' has status {1}, waiting for {2}".format(self.original.name, self.original.status, self.succeed_status)
            if self.original.status == self.succeed_status:
                if self.original.status == common.Status.RUNNING:
                    self.original.stop()
                return common.Status.SUCCESS
            else:
                return common.Status.RUNNING
        return wrapped

    Condition = create_imposter(cls)
    setattr(Condition, "__name__", "Condition<{0}>".format(cls.__name__))
    setattr(Condition, "__init__", _init(Condition.__init__, status))
    setattr(Condition, "update", _update(Condition.update))
    return Condition
