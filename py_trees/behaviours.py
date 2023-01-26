#!/usr/bin/env python
#
# License: BSD
#   https://raw.githubusercontent.com/splintered-reality/py_trees/devel/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""A library of fundamental behaviours for use."""

##############################################################################
# Imports
##############################################################################

import copy
import functools
import operator
import typing

from . import behaviour, blackboard, common, meta

##############################################################################
# Function Behaviours
##############################################################################


def success(self: behaviour.Behaviour) -> common.Status:
    """Define a functor for an always succeeding behaviour.

    Args:
        self: behaviour for this function to substitute update() in.

    Returns:
        behaviour status
    """
    self.logger.debug("%s.update()" % self.__class__.__name__)
    self.feedback_message = "success"
    return common.Status.SUCCESS


def failure(self: behaviour.Behaviour) -> common.Status:
    """Define a functor for an always failing behaviour.

    Args:
        self: behaviour for this function to substitute update() in.

    Returns:
        behaviour status
    """
    self.logger.debug("%s.update()" % self.__class__.__name__)
    self.feedback_message = "failure"
    return common.Status.FAILURE


def running(self: behaviour.Behaviour) -> common.Status:
    """Define a functor for an always running behaviour.

    Args:
        self: behaviour for this function to substitute update() in.

    Returns:
        behaviour status
    """
    self.logger.debug("%s.update()" % self.__class__.__name__)
    self.feedback_message = "running"
    return common.Status.RUNNING


def dummy(self: behaviour.Behaviour) -> common.Status:
    """Define a functor for a crash test dummy behaviour.

    Args:
        self: behaviour for this function to substitute update() in.

    Returns:
        behaviour status
    """
    self.logger.debug("%s.update()" % self.__class__.__name__)
    self.feedback_message = "crash test dummy"
    return common.Status.RUNNING


Success = meta.create_behaviour_from_function(success, "py_trees.behaviours")
"""
Do nothing but tick over with :data:`~py_trees.common.Status.SUCCESS`.
"""

Failure = meta.create_behaviour_from_function(failure, "py_trees.behaviours")
"""
Do nothing but tick over with :data:`~py_trees.common.Status.FAILURE`.
"""

Running = meta.create_behaviour_from_function(running, "py_trees.behaviours")
"""
Do nothing but tick over with :data:`~py_trees.common.Status.RUNNING`.
"""

Dummy = meta.create_behaviour_from_function(dummy, "py_trees.behaviours")
"""
Crash test dummy used for anything dangerous.
"""

##############################################################################
# Standalone Behaviours
##############################################################################


class Periodic(behaviour.Behaviour):
    """
    Simply periodically rotates it's status over all each status.

    That is, :data:`~py_trees.common.Status.RUNNING` for N ticks,
    :data:`~py_trees.common.Status.SUCCESS` for N ticks,
    :data:`~py_trees.common.Status.FAILURE` for N ticks...

    Args:
        name: name of the behaviour
        n: period value (in ticks)

    .. note:: It does not reset the count when initialising.
    """

    def __init__(self, name: str, n: int):
        super(Periodic, self).__init__(name)
        self.count = 0
        self.period = n
        self.response = common.Status.RUNNING

    def update(self) -> common.Status:
        """
        Increment counter and use to decide the current status.

        Returns:
            the behaviour's new status :class:`~py_trees.common.Status`
        """
        self.count += 1
        if self.count > self.period:
            if self.response == common.Status.FAILURE:
                self.feedback_message = "flip to running"
                self.response = common.Status.RUNNING
            elif self.response == common.Status.RUNNING:
                self.feedback_message = "flip to success"
                self.response = common.Status.SUCCESS
            else:
                self.feedback_message = "flip to failure"
                self.response = common.Status.FAILURE
            self.count = 0
        else:
            self.feedback_message = "constant"
        return self.response


class StatusQueue(behaviour.Behaviour):
    """
    Cycle through a specified queue of states.

    .. note::

       This does not reset when the behaviour initialises.

    Args:
        name: name of the behaviour
        sequence: list of status values to cycle through
        eventually: status to use eventually, None to re-cycle the sequence
    """

    def __init__(
        self,
        name: str,
        queue: typing.List[common.Status],
        eventually: typing.Optional[common.Status],
    ):
        super(StatusQueue, self).__init__(name)
        self.queue = queue
        self.eventually = eventually
        self.current_queue = copy.copy(queue)

    def update(self) -> common.Status:
        """
        Pop from the queue or rotate / switch to eventual if the end has been reached.

        Returns:
            the :class:`~py_trees.common.Status` from the popped queue / eventual element
        """
        self.logger.debug("%s.update()" % (self.__class__.__name__))
        if self.current_queue:
            status = self.current_queue.pop(0)
        elif self.eventually is not None:
            status = self.eventually
        else:
            self.current_queue = copy.copy(self.queue)
            status = self.current_queue.pop(0)
        return status

    def terminate(self, new_status: common.Status) -> None:
        """
        Log debug information.

        Args:
            new_status: the behaviour is transitioning to this new status
        """
        self.logger.debug(
            "%s.terminate(%s->%s)" % (self.__class__.__name__, self.status, new_status)
        )


class SuccessEveryN(behaviour.Behaviour):
    """
    Non-blocking, periodic success.

    This behaviour updates it's status with :data:`~py_trees.common.Status.SUCCESS`
    once every N ticks, :data:`~py_trees.common.Status.FAILURE` otherwise.

    Args:
        name: name of the behaviour
        n: trigger success on every n'th tick

    .. tip::
       Use with decorators to change the status value as desired, e.g.
       :meth:`py_trees.decorators.FailureIsRunning`
    """

    def __init__(self, name: str, n: int):
        super(SuccessEveryN, self).__init__(name)
        self.count = 0
        self.every_n = n

    def update(self) -> common.Status:
        """
        Increment the counter and decide on success/failure from that.

        Returns:
            :data:`~py_trees.common.Status.SUCCESS` if the nth tick, :data:`~py_trees.common.Status.FAILURE` otherwise.
        """
        self.count += 1
        self.logger.debug("%s.update()][%s]" % (self.__class__.__name__, self.count))
        if self.count % self.every_n == 0:
            self.feedback_message = "now"
            return common.Status.SUCCESS
        else:
            self.feedback_message = "not yet"
            return common.Status.FAILURE


class TickCounter(behaviour.Behaviour):
    """
    Block for a specified tick count.

    A useful utility behaviour for demos and tests. Simply
    ticks with :data:`~py_trees.common.Status.RUNNING` for
    the specified number of ticks before returning the
    requested completion status (:data:`~py_trees.common.Status.SUCCESS`
    or :data:`~py_trees.common.Status.FAILURE`).

    This behaviour will reset the tick counter when initialising.

    Args:
        name: name of the behaviour
        duration: number of ticks to run
        completion_status: status to switch to once the counter has expired
    """

    def __init__(self, name: str, duration: int, completion_status: common.Status):
        super().__init__(name=name)
        self.completion_status = completion_status
        self.duration = duration
        self.counter = 0

    def initialise(self) -> None:
        """Reset the tick counter."""
        self.counter = 0

    def update(self) -> common.Status:
        """
        Increment the tick counter and check to see if it should complete.

        Returns
            :data:`~py_trees.common.Status.RUNNING` while not expired, the given completion status otherwise
        """
        self.counter += 1
        if self.counter <= self.duration:
            return common.Status.RUNNING
        else:
            return self.completion_status


##############################################################################
# Blackboard Behaviours
##############################################################################


class BlackboardToStatus(behaviour.Behaviour):
    """
    Reflects a :py:data:`~py_trees.common.Status` stored in a blackboard variable.

    This behaviour reverse engineers the :class:`~py_trees.decorators.StatusToBlackboard`
    decorator. Used in conjuction with that decorator, this behaviour can be used to
    reflect the status of a decision elsewhere in the tree.

    .. note::

       A word of caution. The consequences of a behaviour's status should be discernable
       upon inspection of the tree graph. If using StatusToBlackboard
       and BlackboardToStatus to reflect a behaviour's status across a tree,
       this is no longer true. The graph of the tree communicates the local consequences,
       but not the reflected consequences at the point BlackboardToStatus is used. A
       recommendation, use this class only where other options are infeasible or impractical.

    Args:
        variable_name: name of the variable look for, may be nested, e.g. battery.percentage
        name: name of the behaviour

    Raises:
        KeyError: if the variable doesn't exist
        TypeError: if the variable isn't of type :py:data:`~py_trees.common.Status`
    """

    def __init__(
        self,
        name: str,
        variable_name: str,
    ):
        super().__init__(name=name)
        name_components = variable_name.split(".")
        self.key = name_components[0]
        self.key_attributes = ".".join(
            name_components[1:]
        )  # empty string if no other parts
        self.variable_name = variable_name
        self.blackboard = self.attach_blackboard_client()
        self.blackboard.register_key(key=self.key, access=common.Access.READ)

    def update(self) -> common.Status:
        """
        Check for existence.

        Returns:
             :data:`~py_trees.common.Status.SUCCESS` if key found, :data:`~py_trees.common.Status.FAILURE` otherwise.
        """
        self.logger.debug("%s.update()" % self.__class__.__name__)
        # raises a KeyError if the variable doesn't exist
        status = self.blackboard.get(self.variable_name)
        if not isinstance(status, common.Status):
            raise TypeError(
                f"{self.variable_name} is not of type py_trees.common.Status"
            )
        self.feedback_message = f"{self.variable_name}: {status}"
        return status


class CheckBlackboardVariableExists(behaviour.Behaviour):
    """
    A non-blocking check for the existence of a blackboard variable.

    Check the blackboard to verify if a specific variable (key-value pair)
    exists. This is non-blocking, so will always tick with
    status :data:`~py_trees.common.Status.FAILURE`
    :data:`~py_trees.common.Status.SUCCESS`.

    .. seealso::

       :class:`~py_trees.behaviours.WaitForBlackboardVariable` for
       the blocking counterpart to this behaviour.

    Args:
        variable_name: name of the variable look for, may be nested, e.g. battery.percentage
        name: name of the behaviour
    """

    def __init__(
        self,
        name: str,
        variable_name: str,
    ):
        super().__init__(name=name)
        self.variable_name = variable_name
        name_components = variable_name.split(".")
        self.key = name_components[0]
        self.key_attributes = ".".join(
            name_components[1:]
        )  # empty string if no other parts
        self.blackboard = self.attach_blackboard_client()
        self.blackboard.register_key(key=self.key, access=common.Access.READ)

    def update(self) -> common.Status:
        """
        Check for existence.

        Returns:
             :data:`~py_trees.common.Status.SUCCESS` if key found, :data:`~py_trees.common.Status.FAILURE` otherwise.
        """
        self.logger.debug("%s.update()" % self.__class__.__name__)
        try:
            _ = self.blackboard.get(self.variable_name)
            self.feedback_message = "variable '{}' found".format(self.variable_name)
            return common.Status.SUCCESS
        except KeyError:
            self.feedback_message = "variable '{}' not found".format(self.variable_name)
            return common.Status.FAILURE


class WaitForBlackboardVariable(CheckBlackboardVariableExists):
    """
    Block until a blackboard variable comes into existence.

    This is blocking, so it will tick with
    status :data:`~py_trees.common.Status.SUCCESS` if the variable is found,
    and :data:`~py_trees.common.Status.RUNNING` otherwise.

    .. seealso::

       :class:`~py_trees.behaviours.CheckBlackboardVariableExists` for
       the non-blocking counterpart to this behaviour.

    Args:
        variable_name: name of the variable to wait for, may be nested, e.g. battery.percentage
        name: name of the behaviour
    """

    def __init__(
        self,
        name: str,
        variable_name: str,
    ):
        super().__init__(name=name, variable_name=variable_name)

    def update(self) -> common.Status:
        """
        Check for existence, wait otherwise.

        Returns:
             :data:`~py_trees.common.Status.SUCCESS` if key found, :data:`~py_trees.common.Status.RUNNING` otherwise.
        """
        self.logger.debug("%s.update()" % self.__class__.__name__)
        new_status = super().update()
        # CheckBlackboardExists only returns SUCCESS || FAILURE
        if new_status == common.Status.SUCCESS:
            self.feedback_message = "'{}' found".format(self.key)
            return common.Status.SUCCESS
        else:  # new_status == common.Status.FAILURE
            self.feedback_message = "waiting for key '{}'...".format(self.key)
            return common.Status.RUNNING


class UnsetBlackboardVariable(behaviour.Behaviour):
    """
    Unset the specified variable (key-value pair) from the blackboard.

    This always returns
    :data:`~py_trees.common.Status.SUCCESS` regardless of whether
    the variable was already present or not.

    Args:
        key: unset this key-value pair
        name: name of the behaviour
    """

    def __init__(self, name: str, key: str):
        super().__init__(name=name)
        self.key = key
        self.blackboard = self.attach_blackboard_client()
        self.blackboard.register_key(key=self.key, access=common.Access.WRITE)

    def update(self) -> common.Status:
        """
        Unset and always return success.

        Returns:
             :data:`~py_trees.common.Status.SUCCESS`
        """
        if self.blackboard.unset(self.key):
            self.feedback_message = "'{}' found and removed".format(self.key)
        else:
            self.feedback_message = "'{}' not found, nothing to remove"
        return common.Status.SUCCESS


class SetBlackboardVariable(behaviour.Behaviour):
    """
    Set the specified variable on the blackboard.

    Args:
        variable_name: name of the variable to set, may be nested, e.g. battery.percentage
        variable_value: value of the variable to set
        overwrite: when False, do not set the variable if it already exists
        name: name of the behaviour
    """

    def __init__(
        self,
        name: str,
        variable_name: str,
        variable_value: typing.Union[typing.Any, typing.Callable[[], typing.Any]],
        overwrite: bool,
    ):
        super().__init__(name=name)
        self.variable_name = variable_name
        name_components = variable_name.split(".")
        self.key = name_components[0]
        self.key_attributes = ".".join(
            name_components[1:]
        )  # empty string if no other parts
        self.blackboard = self.attach_blackboard_client()
        self.blackboard.register_key(key=self.key, access=common.Access.WRITE)
        self.variable_value_generator = (
            variable_value if callable(variable_value) else lambda: variable_value
        )
        self.overwrite = overwrite

    def update(self) -> common.Status:
        """
        Attempt to set the stored value in the requested blackboard variable.

        Returns:
             :data:`~py_trees.common.Status.FAILURE` if no overwrite requested
                 and the variable exists,  :data:`~py_trees.common.Status.SUCCESS` otherwise
        """
        if self.blackboard.set(
            self.variable_name,
            self.variable_value_generator(),
            overwrite=self.overwrite,
        ):
            return common.Status.SUCCESS
        else:
            return common.Status.FAILURE


class CheckBlackboardVariableValue(behaviour.Behaviour):
    """
    Non-blocking check to determine if a blackboard variable matches a given value/expression.

    Inspect a blackboard variable and if it exists, check that it
    meets the specified criteria (given by operation type and expected value).
    This is non-blocking, so it will always tick with
    :data:`~py_trees.common.Status.SUCCESS` or
    :data:`~py_trees.common.Status.FAILURE`.

    Args:
        name: name of the behaviour
        check: a comparison expression to check against

    .. note::
        If the variable does not yet exist on the blackboard, the behaviour will
        return with status :data:`~py_trees.common.Status.FAILURE`.

    .. tip::
        The python `operator module`_ includes many useful comparison operations.
    """

    def __init__(self, name: str, check: common.ComparisonExpression):
        super().__init__(name=name)
        self.check = check
        name_components = self.check.variable.split(".")
        self.key = name_components[0]
        self.key_attributes = ".".join(
            name_components[1:]
        )  # empty string if no other parts
        self.blackboard = self.attach_blackboard_client()
        self.blackboard.register_key(key=self.key, access=common.Access.READ)

    def update(self) -> common.Status:
        """
        Check for existence, or the appropriate match on the expected value.

        Returns:
             :class:`~py_trees.common.Status`: :data:`~py_trees.common.Status.FAILURE`
                 if not matched, :data:`~py_trees.common.Status.SUCCESS` otherwise.
        """
        self.logger.debug("%s.update()" % self.__class__.__name__)
        try:
            value = self.blackboard.get(self.key)
            if self.key_attributes:
                try:
                    value = operator.attrgetter(self.key_attributes)(value)
                except AttributeError:
                    self.feedback_message = (
                        "blackboard key-value pair exists, but the value does not "
                        f"have the requested nested attributes [{self.key}]"
                    )
                    return common.Status.FAILURE
        except KeyError:
            self.feedback_message = (
                "key '{}' does not yet exist on the blackboard".format(
                    self.check.variable
                )
            )
            return common.Status.FAILURE

        success = self.check.operator(value, self.check.value)

        if success:
            self.feedback_message = "'%s' comparison succeeded [v: %s][e: %s]" % (
                self.check.variable,
                value,
                self.check.value,
            )
            return common.Status.SUCCESS
        else:
            self.feedback_message = "'%s' comparison failed [v: %s][e: %s]" % (
                self.check.variable,
                value,
                self.check.value,
            )
            return common.Status.FAILURE


class WaitForBlackboardVariableValue(CheckBlackboardVariableValue):
    """
    Block until a blackboard variable matches a given value/expression.

    Inspect a blackboard variable and if it exists, check that it
    meets the specified criteria (given by operation type and expected value).
    This is blocking, so it will always tick with
    :data:`~py_trees.common.Status.SUCCESS` or
    :data:`~py_trees.common.Status.RUNNING`.

    .. seealso::

       :class:`~py_trees.behaviours.CheckBlackboardVariableValue` for
       the non-blocking counterpart to this behaviour.

    .. note::
        If the variable does not yet exist on the blackboard, the behaviour will
        return with status :data:`~py_trees.common.Status.RUNNING`.

    Args:
        check: a comparison expression to check against
        name: name of the behaviour
    """

    def __init__(
        self,
        name: str,
        check: common.ComparisonExpression,
    ):
        super().__init__(check=check, name=name)

    def update(self) -> common.Status:
        """
        Check for existence, or the appropriate match on the expected value.

        Returns:
             :class:`~py_trees.common.Status`: :data:`~py_trees.common.Status.FAILURE`
                 if not matched, :data:`~py_trees.common.Status.SUCCESS` otherwise.
        """
        new_status = super().update()
        if new_status == common.Status.FAILURE:
            return common.Status.RUNNING
        else:
            return new_status


class CheckBlackboardVariableValues(behaviour.Behaviour):
    """
    Apply a logical operation across a set of blackboard variable checks.

    This is non-blocking, so will always tick with status
    :data:`~py_trees.common.Status.FAILURE` or
    :data:`~py_trees.common.Status.SUCCESS`.

    Args:
        checks: a list of comparison checks to apply to blackboard variables
        logical_operator: a logical check to apply across the results of the blackboard variable checks
        name: name of the behaviour
        namespace: optionally store results of the checks (boolean) under this namespace

    .. tip::
        The python `operator module`_ includes many useful logical operators, e.g. operator.xor.

    Raises:
        ValueError if less than two variable checks are specified (insufficient for logical operations)
    """

    def __init__(
        self,
        name: str,
        checks: typing.List[common.ComparisonExpression],
        operator: typing.Callable[[bool, bool], bool],
        namespace: typing.Optional[str] = None,
    ):
        super().__init__(name=name)
        self.checks = checks
        self.operator = operator
        self.blackboard = self.attach_blackboard_client()
        if len(checks) < 2:
            raise ValueError(
                "Must be at least two variables to operate on [only {} provided]".format(
                    len(checks)
                )
            )
        for check in self.checks:
            self.blackboard.register_key(
                key=blackboard.Blackboard.key(check.variable), access=common.Access.READ
            )
        self.blackboard_results = None
        if namespace is not None:
            self.blackboard_results = self.attach_blackboard_client(namespace=namespace)
            for counter in range(1, len(self.checks) + 1):
                self.blackboard_results.register_key(
                    key=str(counter), access=common.Access.WRITE
                )

    def update(self) -> common.Status:
        """
        Apply comparison checks on each and a logical check across all variables.

        Returns:
             :data:`~py_trees.common.Status.FAILURE` if key retrieval or logical
                 checks failed, :data:`~py_trees.common.Status.SUCCESS` otherwise.
        """
        self.logger.debug("%s.update()" % self.__class__.__name__)
        results = []
        for check in self.checks:
            try:
                value = self.blackboard.get(check.variable)
            except KeyError:
                self.feedback_message = (
                    "variable '{}' does not yet exist on the blackboard".format(
                        check.variable
                    )
                )
                return common.Status.FAILURE
            results.append(check.operator(value, check.value))
        if self.blackboard_results is not None:
            for counter in range(1, len(results) + 1):
                self.blackboard_results.set(str(counter), results[counter - 1])
        logical_result = functools.reduce(self.operator, results)
        if logical_result:
            self.feedback_message = "[{}]".format(
                "|".join(["T" if result else "F" for result in results])
            )
            return common.Status.SUCCESS
        else:
            self.feedback_message = "[{}]".format(
                "|".join(["T" if result else "F" for result in results])
            )
            return common.Status.FAILURE
