#!/usr/bin/env python
#
# License: BSD
#   https://raw.githubusercontent.com/splintered-reality/py_trees/devel/LICENSE
#
##############################################################################
# Documentation
##############################################################################
"""
A library of fundamental behaviours for use.
"""

##############################################################################
# Imports
##############################################################################

import operator
import typing

from . import behaviour
from . import common
from . import meta

##############################################################################
# Function Behaviours
##############################################################################


def success(self):
    self.logger.debug("%s.update()" % self.__class__.__name__)
    self.feedback_message = "success"
    return common.Status.SUCCESS


def failure(self):
    self.logger.debug("%s.update()" % self.__class__.__name__)
    self.feedback_message = "failure"
    return common.Status.FAILURE


def running(self):
    self.logger.debug("%s.update()" % self.__class__.__name__)
    self.feedback_message = "running"
    return common.Status.RUNNING


def dummy(self):
    self.logger.debug("%s.update()" % self.__class__.__name__)
    self.feedback_message = "crash test dummy"
    return common.Status.RUNNING


Success = meta.create_behaviour_from_function(success)
"""
Do nothing but tick over with :data:`~py_trees.common.Status.SUCCESS`.
"""

Failure = meta.create_behaviour_from_function(failure)
"""
Do nothing but tick over with :data:`~py_trees.common.Status.FAILURE`.
"""

Running = meta.create_behaviour_from_function(running)
"""
Do nothing but tick over with :data:`~py_trees.common.Status.RUNNING`.
"""

Dummy = meta.create_behaviour_from_function(dummy)
"""
Crash test dummy used for anything dangerous.
"""

##############################################################################
# Standalone Behaviours
##############################################################################


class Periodic(behaviour.Behaviour):
    """
    Simply periodically rotates it's status over the
    :data:`~py_trees.common.Status.RUNNING`, :data:`~py_trees.common.Status.SUCCESS`,
    :data:`~py_trees.common.Status.FAILURE` states.
    That is, :data:`~py_trees.common.Status.RUNNING` for N ticks,
    :data:`~py_trees.common.Status.SUCCESS` for N ticks,
    :data:`~py_trees.common.Status.FAILURE` for N ticks...

    Args:
        name (:obj:`str`): name of the behaviour
        n (:obj:`int`): period value (in ticks)

    .. note:: It does not reset the count when initialising.
    """
    def __init__(self, name, n):
        super(Periodic, self).__init__(name)
        self.count = 0
        self.period = n
        self.response = common.Status.RUNNING

    def update(self):
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


class SuccessEveryN(behaviour.Behaviour):
    """
    This behaviour updates it's status with :data:`~py_trees.common.Status.SUCCESS`
    once every N ticks, :data:`~py_trees.common.Status.FAILURE` otherwise.

    Args:
        name (:obj:`str`): name of the behaviour
        n (:obj:`int`): trigger success on every n'th tick

    .. tip::
       Use with decorators to change the status value as desired, e.g.
       :meth:`py_trees.decorators.FailureIsRunning`
    """
    def __init__(self, name, n):
        super(SuccessEveryN, self).__init__(name)
        self.count = 0
        self.every_n = n

    def update(self):
        self.count += 1
        self.logger.debug("%s.update()][%s]" % (self.__class__.__name__, self.count))
        if self.count % self.every_n == 0:
            self.feedback_message = "now"
            return common.Status.SUCCESS
        else:
            self.feedback_message = "not yet"
            return common.Status.FAILURE


class Count(behaviour.Behaviour):
    """
    A counting behaviour that updates its status at each tick depending on
    the value of the counter. The status will move through the states in order -
    :data:`~py_trees.common.Status.FAILURE`, :data:`~py_trees.common.Status.RUNNING`,
    :data:`~py_trees.common.Status.SUCCESS`.

    This behaviour is useful for simple testing and demo scenarios.

    Args:
        name (:obj:`str`): name of the behaviour
        fail_until (:obj:`int`): set status to :data:`~py_trees.common.Status.FAILURE` until the counter reaches this value
        running_until (:obj:`int`): set status to :data:`~py_trees.common.Status.RUNNING` until the counter reaches this value
        success_until (:obj:`int`): set status to :data:`~py_trees.common.Status.SUCCESS` until the counter reaches this value
        reset (:obj:`bool`): whenever invalidated (usually by a sequence reinitialising, or higher priority interrupting)

    Attributes:
        count (:obj:`int`): a simple counter which increments every tick
    """
    def __init__(self, name="Count", fail_until=3, running_until=5, success_until=6, reset=True):
        super(Count, self).__init__(name)
        self.count = 0
        self.fail_until = fail_until
        self.running_until = running_until
        self.success_until = success_until
        self.number_count_resets = 0
        self.number_updated = 0
        self.reset = reset

    def terminate(self, new_status):
        self.logger.debug("%s.terminate(%s->%s)" % (self.__class__.__name__, self.status, new_status))
        # reset only if udpate got us into an invalid state
        if new_status == common.Status.INVALID and self.reset:
            self.count = 0
            self.number_count_resets += 1
        self.feedback_message = ""

    def update(self):
        self.number_updated += 1
        self.count += 1
        if self.count <= self.fail_until:
            self.logger.debug("%s.update()[%s: failure]" % (self.__class__.__name__, self.count))
            self.feedback_message = "failing"
            return common.Status.FAILURE
        elif self.count <= self.running_until:
            self.logger.debug("%s.update()[%s: running]" % (self.__class__.__name__, self.count))
            self.feedback_message = "running"
            return common.Status.RUNNING
        elif self.count <= self.success_until:
            self.logger.debug("%s.update()[%s: success]" % (self.__class__.__name__, self.count))
            self.feedback_message = "success"
            return common.Status.SUCCESS
        else:
            self.logger.debug("%s.update()[%s: failure]" % (self.__class__.__name__, self.count))
            self.feedback_message = "failing forever more"
            return common.Status.FAILURE

    def __repr__(self):
        """
        Simple string representation of the object.

        Returns:
            :obj:`str`: string representation
        """
        s = "%s\n" % self.name
        s += "  Status : %s\n" % self.status
        s += "  Count  : %s\n" % self.count
        s += "  Resets : %s\n" % self.number_count_resets
        s += "  Updates: %s\n" % self.number_updated
        return s

##############################################################################
# Blackboard Behaviours
##############################################################################


class CheckBlackboardVariableExists(behaviour.Behaviour):
    """
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
            variable_name: str,
            name: str=common.Name.AUTO_GENERATED
    ):
        super().__init__(name=name)
        self.variable_name = variable_name
        name_components = variable_name.split('.')
        self.key = name_components[0]
        self.key_attributes = '.'.join(name_components[1:])  # empty string if no other parts
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
            unused_value = self.blackboard.get(self.variable_name)
            self.feedback_message = "variable '{}' found".format(self.variable_name)
            return common.Status.SUCCESS
        except KeyError:
            self.feedback_message = "variable '{}' not found".format(self.variable_name)
            return common.Status.FAILURE


class WaitForBlackboardVariable(CheckBlackboardVariableExists):
    """
    Wait for the blackboard variable to become available on the blackboard.
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
            variable_name: str,
            name: str=common.Name.AUTO_GENERATED
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
        if new_status == common.Status.SUCCESS:
            self.feedback_message = "'{}' found".format(self.key)
            return common.Status.SUCCESS
        elif new_status == common.Status.FAILURE:
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
    def __init__(self,
                 key: str,
                 name: str=common.Name.AUTO_GENERATED,
                 ):
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
            variable_name: str,
            variable_value: typing.Any,
            overwrite: bool = True,
            name: str=common.Name.AUTO_GENERATED,
    ):
        super().__init__(name=name)
        self.variable_name = variable_name
        name_components = variable_name.split('.')
        self.key = name_components[0]
        self.key_attributes = '.'.join(name_components[1:])  # empty string if no other parts
        self.blackboard = self.attach_blackboard_client()
        self.blackboard.register_key(key=self.key, access=common.Access.WRITE)
        self.variable_value = variable_value
        self.overwrite = overwrite

    def update(self) -> common.Status:
        """
        Always return success.

        Returns:
             :data:`~py_trees.common.Status.FAILURE` if no overwrite requested and the variable exists,  :data:`~py_trees.common.Status.SUCCESS` otherwise
        """
        if self.blackboard.set(
            self.variable_name,
            self.variable_value,
            overwrite=self.overwrite
        ):
            return common.Status.SUCCESS
        else:
            return common.Status.FAILURE


class CheckBlackboardVariableValue(behaviour.Behaviour):
    """
    Inspect a blackboard variable and if it exists, check that it
    meets the specified criteria (given by operation type and expected value).
    This is non-blocking, so it will always tick with
    :data:`~py_trees.common.Status.SUCCESS` or
    :data:`~py_trees.common.Status.FAILURE`.

    Args:
        variable_name: name of the variable to check, may be nested, e.g. battery.percentage
        expected_value: expected value
        comparison_operator: any method that can compare the value against the expected value
        name: name of the behaviour

    .. note::
        If the variable does not yet exist on the blackboard, the behaviour will
        return with status :data:`~py_trees.common.Status.FAILURE`.

    .. tip::
        The python `operator module`_ includes many useful comparison operations.

    .. _`operator module`: https://docs.python.org/2/library/operator.html
    """
    def __init__(
            self,
            variable_name: str,
            expected_value: typing.Any,
            comparison_operator: typing.Callable[[typing.Any, typing.Any], bool]=operator.eq,
            name: str=common.Name.AUTO_GENERATED
    ):
        super().__init__(name=name)
        self.variable_name = variable_name
        name_components = variable_name.split('.')
        self.key = name_components[0]
        self.key_attributes = '.'.join(name_components[1:])  # empty string if no other parts
        self.blackboard = self.attach_blackboard_client()
        self.blackboard.register_key(key=self.key, access=common.Access.READ)

        self.expected_value = expected_value
        self.comparison_operator = comparison_operator

    def update(self):
        """
        Check for existence, or the appropriate match on the expected value.

        Returns:
             :class:`~py_trees.common.Status`: :data:`~py_trees.common.Status.FAILURE` if not matched, :data:`~py_trees.common.Status.SUCCESS` otherwise.
        """
        self.logger.debug("%s.update()" % self.__class__.__name__)
        try:
            value = self.blackboard.get(self.key)
            if self.key_attributes:
                try:
                    value = operator.attrgetter(self.key_attributes)(value)
                except AttributeError:
                    self.feedback_message = 'blackboard key-value pair exists, but the value does not have the requested nested attributes [{}]'.format(self.variable_name)
                    return common.Status.FAILURE
        except KeyError:
            self.feedback_message = "key '{}' does not yet exist on the blackboard".format(self.variable_name)
            return common.Status.FAILURE

        success = self.comparison_operator(value, self.expected_value)

        if success:
            self.feedback_message = "'%s' comparison succeeded [v: %s][e: %s]" % (self.variable_name, value, self.expected_value)
            return common.Status.SUCCESS
        else:
            self.feedback_message = "'%s' comparison failed [v: %s][e: %s]" % (self.variable_name, value, self.expected_value)
            return common.Status.FAILURE


class WaitForBlackboardVariableValue(CheckBlackboardVariableValue):
    """
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
        variable_name: name of the variable to check, may be nested, e.g. battery.percentage
        expected_value: expected value
        comparison_operator: any method that can compare the value against the expected value
        name: name of the behaviour
    """
    def __init__(
            self,
            variable_name: str,
            expected_value: typing.Any,
            comparison_operator: typing.Callable[[typing.Any, typing.Any], bool]=operator.eq,
            name: str=common.Name.AUTO_GENERATED
    ):
        super().__init__(
            variable_name=variable_name,
            expected_value=expected_value,
            comparison_operator=comparison_operator,
            name=name
        )

    def update(self):
        """
        Check for existence, or the appropriate match on the expected value.

        Returns:
             :class:`~py_trees.common.Status`: :data:`~py_trees.common.Status.FAILURE` if not matched, :data:`~py_trees.common.Status.SUCCESS` otherwise.
        """
        new_status = super().update()
        if new_status == common.Status.FAILURE:
            return common.Status.RUNNING
        else:
            return new_status
