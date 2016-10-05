#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/stonier/py_trees_suite/license/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
.. module:: common
   :platform: Unix
   :synopsis: Shared data storage for py_trees behaviours.

Oh my spaghettified magnificence,
Bless my noggin with a tickle from your noodly appendages!
"""

##############################################################################
# Imports
##############################################################################

import operator
import rocon_console.console as console

from . import behaviours
from . import common

##############################################################################
# Classes
##############################################################################


class Blackboard(object):
    """
      Borg style data store for sharing amongst behaviours.

      http://code.activestate.com/recipes/66531-singleton-we-dont-need-no-stinkin-singleton-the-bo/

      To register new variables on the blackboard, just promiscuously do so from instantiations of
      the borg. e.g.

      @code
        blackboard = Blackboard()
        blackboard.foo = "bar"
      @endcode

      This has the problem that it could collide with an existing key that is being used by another
      behaviour (independently). If we wanted to safeguard against these collisions, we should
      think some more. Perhaps overriding __setattr__ to do the check for us could work, but it means
      it will trigger the check even when you are setting the variable. Probably we want a much
      more complex base borg object to enable this kind of rigour.

      Convenience or Rigour?
    """
    __shared_state = {}

    def __init__(self):
        self.__dict__ = self.__shared_state

    def set(self, name, value, overwrite=True):
        """
        For when you only have strings to identify and access the blackboard variables, this
        provides a convenient setter.
        """
        if not overwrite:
            try:
                getattr(self, name)
                return False
            except AttributeError:
                pass
        setattr(self, name, value)
        return True

    def get(self, name):
        """
        For when you only have strings to identify and access the blackboard variables,
        this provides a convenient accessor.
        """
        try:
            return getattr(self, name)
        except AttributeError:
            return None

    def __str__(self):
        s = console.green + type(self).__name__ + "\n" + console.reset
        max_length = 0
        for k in self.__dict__.keys():
            max_length = len(k) if len(k) > max_length else max_length
        keys = sorted(self.__dict__)
        for key in keys:
            value = self.__dict__[key]
            if value is None:
                value_string = "-"
                s += console.cyan + "  " + '{0: <{1}}'.format(key, max_length + 1) + console.reset + ": " + console.yellow + "{0}\n".format(value_string) + console.reset
            else:
                lines = ('{0}'.format(value)).split('\n')
                if len(lines) > 1:
                    s += console.cyan + "  " + '{0: <{1}}'.format(key, max_length + 1) + console.reset + ":\n"
                    for line in lines:
                        s += console.yellow + "    {0}\n".format(line) + console.reset
                else:
                    s += console.cyan + "  " + '{0: <{1}}'.format(key, max_length + 1) + console.reset + ": " + console.yellow + '{0}\n'.format(value) + console.reset
        s += console.reset
        return s


class ClearBlackboardVariable(behaviours.Success):
    """
    Clear the specified value from the blackboard.
    """
    def __init__(self,
                 name="Clear Blackboard Variable",
                 variable_name="dummy",
                 ):
        """
        :param name: name of the behaviour
        :param variable_name: name of the variable to clear
        """
        super(ClearBlackboardVariable, self).__init__(name)
        self.variable_name = variable_name

    def initialise(self):
        self.blackboard = Blackboard()
        try:
            delattr(self.blackboard, self.variable_name)
        except AttributeError:
            pass


class SetBlackboardVariable(behaviours.Success):
    """
    Set the specified variable on the blackboard.
    Usually we set variables from inside other behaviours, but can
    be convenient to set them from a behaviour of their own sometimes so you
    don't get blackboard logic mixed up with more atomic behaviours.

    .. todo:: overwrite option, leading to possible failure/success logic.
    """
    def __init__(self,
                 name="Set Blackboard Variable",
                 variable_name="dummy",
                 variable_value=None
                 ):
        """
        :param name: name of the behaviour
        :param variable_name: name of the variable to set
        :param value_name: value of the variable to set
        """
        super(SetBlackboardVariable, self).__init__(name)
        self.variable_name = variable_name
        self.variable_value = variable_value

    def initialise(self):
        self.blackboard = Blackboard()
        self.blackboard.set(self.variable_name, self.variable_value, overwrite=True)


class CheckBlackboardVariable(behaviours.Behaviour):
    """
    Check the blackboard to see if it has a specific variable
    and optionally whether that variable has a specific value.
    """
    def __init__(self,
                 name,
                 variable_name="dummy",
                 expected_value=None,
                 comparison_operator=operator.eq,
                 clearing_policy=common.ClearingPolicy.ON_INITIALISE
                 ):
        """
        :param name: name of the behaviour
        :param variable_name: name of the variable to check
        :param expected_value: expected value of the variable, if None it will only check for existence
        :param function comparison_operator: one from the python `operator module`_
        :param clearing_policy: when to clear the data, see :py:class:`~py_trees.common.ClearingPolicy`
        """
        super(CheckBlackboardVariable, self).__init__(name)
        self.blackboard = Blackboard()
        self.variable_name = variable_name
        self.expected_value = expected_value
        self.comparison_operator = comparison_operator
        self.matching_result = None
        self.clearing_policy = clearing_policy

    def initialise(self):
        """
        Clears the internally stored message ready for a new run
        if ``old_data_is_valid`` wasn't set.
        """
        self.logger.debug("  %s [CheckBlackboardVariable::initialise()]" % self.name)
        if self.clearing_policy == common.ClearingPolicy.ON_INITIALISE:
            self.matching_result = None

    def update(self):
        self.logger.debug("  %s [CheckBlackboardVariable::update()]" % self.name)
        if self.matching_result is not None:
            return self.matching_result

        result = None
        check_attr = operator.attrgetter(self.variable_name)

        try:
            value = check_attr(self.blackboard)
            # if existence check required only
            if self.expected_value is None:
                self.feedback_message = "'%s' exists on the blackboard (as required)" % self.variable_name
                result = common.Status.SUCCESS
        except AttributeError:
            self.feedback_message = 'blackboard variable {0} did not exist'.format(self.variable_name)
            result = common.Status.FAILURE

        if result is None:
            # expected value matching
            # value = getattr(self.blackboard, self.variable_name)
            success = self.comparison_operator(value, self.expected_value)

            if success:
                self.feedback_message = "'%s' comparison succeeded [v: %s][e: %s]" % (self.variable_name, value, self.expected_value)
                result = common.Status.SUCCESS
            else:
                self.feedback_message = "'%s' comparison failed [v: %s][e: %s]" % (self.variable_name, value, self.expected_value)
                result = common.Status.FAILURE

        if result == common.Status.SUCCESS and self.clearing_policy == common.ClearingPolicy.ON_SUCCESS:
            self.matching_result = None
        else:
            self.matching_result = result
        return result

    def terminate(self, new_status):
        """
        Always reset the variable if it was invalidated.
        """
        self.logger.debug("  %s [WaitForBlackboardVariable::terminate()][%s->%s]" % (self.name, self.status, new_status))
        if new_status == common.Status.INVALID:
            self.matching_result = None


class WaitForBlackboardVariable(behaviours.Behaviour):
    """
    Check the blackboard to see if it has a specific variable
    and optionally whether that variable has a specific value.
    Unlike :py:class:`~py_trees.blackboard.CheckBlackboardVariable`
    this class will be in a running state until the variable appears
    and (optionally) is matched.
    """
    def __init__(self,
                 name,
                 variable_name="dummy",
                 expected_value=None,
                 comparison_operator=operator.eq,
                 clearing_policy=common.ClearingPolicy.ON_INITIALISE
                 ):
        """
        :param name: name of the behaviour
        :param variable_name: name of the variable to check
        :param expected_value: expected value of the variable, if None it will only check for existence
        :param function comparison_operator: one from the python `operator module`_
        :param clearing_policy: when to clear the data, see :py:class:`~py_trees.common.ClearingPolicy`
        """
        super(WaitForBlackboardVariable, self).__init__(name)
        self.blackboard = Blackboard()
        self.variable_name = variable_name
        self.expected_value = expected_value
        self.comparison_operator = comparison_operator
        self.clearing_policy = clearing_policy
        self.matching_result = None

    def initialise(self):
        """
        Clears the internally stored message ready for a new run
        if ``old_data_is_valid`` wasn't set.
        """
        self.logger.debug("  %s [WaitForBlackboardVariable::initialise()]" % self.name)
        if self.clearing_policy == common.ClearingPolicy.ON_INITIALISE:
            self.matching_result = None

    def update(self):
        self.logger.debug("  %s [WaitForBlackboardVariable::update()]" % self.name)
        if self.matching_result is not None:
            return self.matching_result

        # existence failure check
        if not hasattr(self.blackboard, self.variable_name):
            self.feedback_message = 'blackboard variable {0} did not exist'.format(self.variable_name)
            result = common.Status.RUNNING

        # if existence check required only
        elif self.expected_value is None:
            self.feedback_message = "'%s' exists on the blackboard (as required)" % self.variable_name
            result = common.Status.SUCCESS

        else:
            # expected value matching
            value = getattr(self.blackboard, self.variable_name)
            success = self.comparison_operator(value, self.expected_value)

            if success:
                self.feedback_message = "'%s' comparison succeeded [v: %s][e: %s]" % (self.variable_name, value, self.expected_value)
                result = common.Status.SUCCESS
            else:
                self.feedback_message = "'%s' comparison failed [v: %s][e: %s]" % (self.variable_name, value, self.expected_value)
                result = common.Status.RUNNING

        if result == common.Status.SUCCESS and self.clearing_policy == common.ClearingPolicy.ON_SUCCESS:
            self.matching_result = None
        elif result != common.Status.RUNNING:
            self.matching_result = result
        return result

    def terminate(self, new_status):
        """
        Always reset the variable if it was invalidated.
        """
        self.logger.debug("  %s [WaitForBlackboardVariable::terminate()][%s->%s]" % (self.name, self.status, new_status))
        if new_status == common.Status.INVALID:
            self.matching_result = None
