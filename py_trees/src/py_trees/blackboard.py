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

"""

##############################################################################
# Imports
##############################################################################

from . import behaviours

from . import common
from .behaviour import Behaviour
import rocon_console.console as console
import operator
from cPickle import dumps, loads
import rospy
import std_msgs.msg as std_msgs
import rosnode


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


class ROSBlackboard(object):
    """
    Manages :py:class:`Blackboard <py_trees.blackboard.Blackboard>`.
    Provides methods to initialize SubBlackboards to watch subset of Blackboard variables
    And publishers for publishing when variables in Blackboards or SubBlackboards are changed
    """
    class SubBlackboard(object):
        """
        Container class for a Subblackboard
        Keeps track of variables from Blackboard
        """
        def __init__(self, topic_name, attrs):
            self.blackboard = Blackboard()
            self.topic_name = topic_name
            self.attrs = attrs
            self.dict = {}
            self.cached_dict = {}
            self.publisher = rospy.Publisher("~" + topic_name, std_msgs.String, latch=True, queue_size=2)
            self.connection_established = False

        def update_sub_blackboard(self):
            for attr in self.attrs:
                if '/' in attr:
                    check_attr = operator.attrgetter(".".join(attr.split('/')))
                else:
                    check_attr = operator.attrgetter(attr)
                try:
                    value = check_attr(self.blackboard)
                    self.dict[attr] = value
                except AttributeError:
                    pass

        def is_changed(self):
            self.update_sub_blackboard()
            current_pickle = dumps(self.dict, -1)
            blackboard_changed = current_pickle != dumps(self.cached_dict, -1)
            self.cached_dict = loads(current_pickle)

            return blackboard_changed

        def __str__(self):
            s = "\n"
            max_length = 0
            for k in self.dict.keys():
                max_length = len(k) if len(k) > max_length else max_length
            keys = sorted(self.dict)
            for key in keys:
                value = self.dict[key]
                if value is None:
                    value_string = "-"
                    s += console.cyan + "  " + '{0: <{1}}'.format(key, max_length + 1) + console.reset + ": " + console.yellow + "%s\n" % (value_string) + console.reset
                else:
                    lines = ("%s" % value).split('\n')
                    if len(lines) > 1:
                        s += console.cyan + "  " + '{0: <{1}}'.format(key, max_length + 1) + console.reset + ":\n"
                        for line in lines:
                            s += console.yellow + "    %s\n" % line + console.reset
                    else:
                        s += console.cyan + "  " + '{0: <{1}}'.format(key, max_length + 1) + console.reset + ": " + console.yellow + "%s\n" % (value) + console.reset
            s += console.reset
            return s

    def __init__(self):
        self.blackboard = Blackboard()
        self.cached_blackboard_dict = {}
        self.publisher = rospy.Publisher("~blackboard", std_msgs.String, latch=True, queue_size=2)

        # repo of sub_blackboards
        self.sub_blackboards = []

    def get_nested_keys(self):
        variables = []

        def inner(v, k):
            for attr in dir(type(v)):
                if not isinstance(v, (bool, list, str, int, float)):
                        if not attr.startswith("_"):
                            value = getattr(v, attr)
                            if not callable(value):
                                if not attr.isupper():
                                    variables.append(k + "/" + attr)
                                    inner(value, k + "/" + attr)

        for k, v in self.blackboard.__dict__.items():
            variables.append(k)
            inner(v, k)

        return variables

    def initialize_sub_blackboard(self, attrs, topic_name=None):
        if isinstance(attrs, list):
            if not topic_name:
                topic_name = "sub_blackboard_" + str(len(self.sub_blackboards))

            sub_blackboard = ROSBlackboard.SubBlackboard(topic_name, attrs)
            self.sub_blackboards.append(sub_blackboard)

        return topic_name

    def is_changed(self):
        current_pickle = dumps(self.blackboard.__dict__, -1)
        blackboard_changed = current_pickle != self.cached_blackboard_dict
        self.cached_blackboard_dict = current_pickle

        return blackboard_changed

    def publish_blackboard(self, tree):
        """
        Publishes the blackboard. Should be called at the end of every tick.
        """

        # publish blackboard
        if self.publisher.get_num_connections() > 0:
            if self.is_changed():
                self.publisher.publish("%s" % self.blackboard)

        # publish sub_blackboards
        subs_to_remove = []
        for (i, sub_blackboard) in enumerate(self.sub_blackboards):
            if sub_blackboard.publisher.get_num_connections() > 0:
                if sub_blackboard.is_changed():
                    sub_blackboard.publisher.publish("%s" % sub_blackboard)

                if not sub_blackboard.connection_established:
                    sub_blackboard.connection_established = True
            else:
                if sub_blackboard.connection_established:
                    # unregister publisher
                    sub_blackboard.publisher.unregister()

                    # add this to removing list
                    subs_to_remove.append(i)

        # remove stale sub_blackboards
        subs_to_remove = [(x - i) for (i, x) in enumerate(subs_to_remove)]
        for sub_index in subs_to_remove:
            self.sub_blackboards.pop(sub_index)


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


class CheckBlackboardVariable(Behaviour):
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


class WaitForBlackboardVariable(Behaviour):
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
