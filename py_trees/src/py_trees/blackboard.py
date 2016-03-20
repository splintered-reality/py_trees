#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/yujinrobot/gopher_crazy_hospital/license/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
.. module:: common
   :platform: Unix
   :synopsis: Shared data storage for py_trees behaviours.

----

"""

##############################################################################
# Imports
##############################################################################

from .common import Status
from .behaviour import Behaviour
import rocon_console.console as console

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

    def __str__(self):
        s = console.green + type(self).__name__ + "\n" + console.reset
        max_length = 0
        for k in self.__dict__.keys():
            max_length = len(k) if len(k) > max_length else max_length
        for key, value in self.__dict__.iteritems():
            s += console.cyan + "  " + '{0: <{1}}'.format(key, max_length + 1) + console.reset + ": " + console.yellow + "%s\n" % (value if value is not None else '-')
        s += console.reset
        return s


class CheckBlackboardVariable(Behaviour):
    """
    Check the blackboard to see if it has a specific variable
    and optionally whether that variable has a specific value.
    """
    def __init__(self,
                 name,
                 variable_name="dummy",
                 expected_value=None,
                 invert=False
                 ):
        """
        :param name: name of the behaviour
        :param variable_name: name of the variable to check
        :param expected_value: expected value of the variable, if None it will only check for existence
        :param invert: if true, check that the value of the variable is != expected_value, rather than ==
        """
        super(CheckBlackboardVariable, self).__init__(name)
        self.blackboard = Blackboard()
        self.variable_name = variable_name
        self.expected_value = expected_value
        # if the user sets the expected value, we check it
        self.invert = invert

    def update(self):
        # existence failure check
        if not hasattr(self.blackboard, self.variable_name):
            self.feedback_message = 'blackboard variable {0} did not exist'.format(self.variable_name)
            return Status.FAILURE

        # if existence check required only
        if self.expected_value is None:
            self.feedback_message = "'%s' exists on the blackboard (as required)" % self.variable_name
            return Status.SUCCESS

        # expected value matching
        value = getattr(self.blackboard, self.variable_name)
        matched_expected = (value == self.expected_value)

        # result
        if self.invert:
            result = not matched_expected
            if result:
                self.feedback_message = "'%s' did not match expected value (as required) [v: %s][e: %s]" % (self.variable_name, value, self.expected_value)
                return Status.SUCCESS
            else:
                self.feedback_message = "'%s' matched expected value (required otherwise) [v: %s][e: %s]" % (self.variable_name, value, self.expected_value)
                return Status.FAILURE
        else:
            if matched_expected:
                self.feedback_message = "'%s' matched expected value (as required) [v: %s][e: %s]" % (self.variable_name, value, self.expected_value)
                return Status.SUCCESS
            else:
                self.feedback_message = "'%s' did not match expected value (required otherwise) [v: %s][e: %s]" % (self.variable_name, value, self.expected_value)
                return Status.FAILURE
