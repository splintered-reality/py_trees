#!/usr/bin/env python
#
# License: BSD
#   https://raw.githubusercontent.com/stonier/py_trees/devel/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
Blackboards are not a necessary component, but are a fairly standard feature
in most behaviour tree implementations. See, for example, the `design notes`_
for blackboards in Unreal Engine.

.. image:: images/blackboard.jpg
   :width: 300px
   :align: center

Implementations however, tend to vary quite a bit depending on the needs of
the framework using them. Some of the usual considerations include scope
and sharing of blackboards across multiple tree instances.

For this package, we've decided to keep blackboards extremely simple to fit
with the same 'rapid development for small scale systems' principles
that this library is designed for.

* No sharing between tree instances
* No locking for reading/writing
* Global scope, i.e. any behaviour can access any variable
* No external communications (e.g. to a database)

.. include:: weblinks.rst
"""

##############################################################################
# Imports
##############################################################################

import operator

from . import behaviours
from . import common
from . import console

##############################################################################
# Classes
##############################################################################


class Blackboard(object):
    """
    `Borg`_ style key-value store for sharing amongst behaviours.

    .. _Borg: http://code.activestate.com/recipes/66531-singleton-we-dont-need-no-stinkin-singleton-the-bo/

    Examples:
        You can instantiate the blackboard from anywhere in your program. Even
        disconnected calls will get access to the same data store. For example:

        .. code-block:: python

            def check_foo():
                blackboard = Blackboard()
                assert(blackboard.foo, "bar")

            if __name__ == '__main__':
                blackboard = Blackboard()
                blackboard.foo = "bar"
                check_foo()

        If the key value you are interested in is only known at runtime, then
        you can set/get from the blackboard without the convenient variable style
        access:

        .. code-block:: python

            blackboard = Blackboard()
            result = blackboard.set("foo", "bar")
            foo = blackboard.get("foo")

        The blackboard can also be converted and printed (with highlighting)
        as a string. This is useful for logging and debugging.

        .. code-block:: python

            print(Blackboard())


    .. warning::

       Be careful of key collisions. This implementation leaves this management up to the user.

    .. seealso:: The :ref:`py-trees-demo-blackboard-program` program demos use of the blackboard along with a couple of the blackboard behaviours.
    """
    __shared_state = {}

    def __init__(self):
        self.__dict__ = self.__shared_state

    def set(self, name, value, overwrite=True):
        """
        For when you only have strings to identify and access the blackboard variables, this
        provides a convenient setter.

        Args:
            name (:obj:`str`): name of the variable to set
            value (:obj:`any`): any variable type
            overwrite(:obj:`bool`): whether to abort if the value is already present

        Returns:
            :obj:`bool`: always True unless overwrite was set to False and a variable already exists
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

        Args:
            name (:obj:`str`): name of the variable to set
        """
        try:
            return getattr(self, name)
        except AttributeError:
            return None

    def __str__(self):
        """
        Express the blackboard contents as a string. Useful for debugging.

        Returns:
            :obj:`str`: blackboard contents
        """
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

    Args:
        name (:obj:`str`): name of the behaviour
        variable_name (:obj:`str`): name of the variable to clear
    """
    def __init__(self,
                 name="Clear Blackboard Variable",
                 variable_name="dummy",
                 ):
        super(ClearBlackboardVariable, self).__init__(name)
        self.variable_name = variable_name

    def initialise(self):
        """
        Delete the variable from the blackboard.
        """
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

    Args:
        name (:obj:`str`): name of the behaviour
        variable_name (:obj:`str`): name of the variable to set
        variable_value (:obj:`any`): value of the variable to set

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
    and optionally whether that variable has an expected value.
    It is a binary behaviour, always updating it's status
    with either :data:`~py_trees.common.Status.SUCCESS` or
    :data:`~py_trees.common.Status.FAILURE` at each tick.

    Args:
        name (:obj:`str`): name of the behaviour
        variable_name (:obj:`str`): name of the variable to set
        expected_value (:obj:`any`): expected value to find (if `None`, check for existence only)
        comparison_operator (:obj:`func`): one from the python `operator module`_
        clearing_policy (:obj:`any`): when to clear the match result, see :py:class:`~py_trees.common.ClearingPolicy`

    .. tip::
        If just checking for existence, use the default argument `expected_value=None`.

    .. tip::
        There are times when you want to get the expected match once and then save
        that result thereafter. For example, to flag once a system has reached a
        subgoal. Use the :data:`~py_trees.common.ClearingPolicy.NEVER` flag to do this.

    .. include:: weblinks.rst
    """
    def __init__(self,
                 name,
                 variable_name="dummy",
                 expected_value=None,
                 comparison_operator=operator.eq,
                 clearing_policy=common.ClearingPolicy.ON_INITIALISE,
                 debug_feedback_message=False
                 ):
        super(CheckBlackboardVariable, self).__init__(name)
        self.blackboard = Blackboard()
        self.variable_name = variable_name
        self.expected_value = expected_value
        self.comparison_operator = comparison_operator
        self.matching_result = None
        self.clearing_policy = clearing_policy
        self.debug_feedback_message = debug_feedback_message

    def initialise(self):
        """
        Clears the internally stored message ready for a new run
        if ``old_data_is_valid`` wasn't set.
        """
        self.logger.debug("%s.initialise()" % self.__class__.__name__)
        if self.clearing_policy == common.ClearingPolicy.ON_INITIALISE:
            self.matching_result = None

    def update(self):
        """
        Check for existence, or the appropriate match on the expected value.

        Returns:
             :class:`~py_trees.common.Status`: :data:`~py_trees.common.Status.FAILURE` if not matched, :data:`~py_trees.common.Status.SUCCESS` otherwise.
        """
        self.logger.debug("%s.update()" % self.__class__.__name__)
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
                if self.debug_feedback_message:  # costly
                    self.feedback_message = "'%s' comparison succeeded [v: %s][e: %s]" % (self.variable_name, value, self.expected_value)
                else:
                    self.feedback_message = "'%s' comparison succeeded" % (self.variable_name)
                result = common.Status.SUCCESS
            else:
                if self.debug_feedback_message:  # costly
                    self.feedback_message = "'%s' comparison failed [v: %s][e: %s]" % (self.variable_name, value, self.expected_value)
                else:
                    self.feedback_message = "'%s' comparison failed" % (self.variable_name)
                result = common.Status.FAILURE

        if result == common.Status.SUCCESS and self.clearing_policy == common.ClearingPolicy.ON_SUCCESS:
            self.matching_result = None
        else:
            self.matching_result = result
        return result

    def terminate(self, new_status):
        """
        Always discard the matching result if it was invalidated by a parent or
        higher priority interrupt.
        """
        self.logger.debug("%s.terminate(%s)" % (self.__class__.__name__, "%s->%s" % (self.status, new_status) if self.status != new_status else "%s" % new_status))
        if new_status == common.Status.INVALID:
            self.matching_result = None


class WaitForBlackboardVariable(behaviours.Behaviour):
    """
    Check the blackboard to see if it has a specific variable
    and optionally whether that variable has a specific value.
    Unlike :py:class:`~py_trees.blackboard.CheckBlackboardVariable`
    this class will be in a :data:`~py_trees.common.Status.RUNNING` state until the variable appears
    and (optionally) is matched.

    Args:
        name (:obj:`str`): name of the behaviour
        variable_name (:obj:`str`): name of the variable to check
        expected_value (:obj:`any`): expected value to find (if `None`, check for existence only)
        comparison_operator (:obj:`func`): one from the python `operator module`_
        clearing_policy (:obj:`any`): when to clear the match result, see :py:class:`~py_trees.common.ClearingPolicy`

    .. tip::
        There are times when you want to get the expected match once and then save
        that result thereafter. For example, to flag once a system has reached a
        subgoal. Use the :data:`~py_trees.common.ClearingPolicy.NEVER` flag to do this.

    .. seealso:: :class:`~py_trees.blackboard.CheckBlackboardVariable`

    .. include:: weblinks.rst
    """
    def __init__(self,
                 name,
                 variable_name="dummy",
                 expected_value=None,
                 comparison_operator=operator.eq,
                 clearing_policy=common.ClearingPolicy.ON_INITIALISE
                 ):
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
        self.logger.debug("%s.initialise()" % self.__class__.__name__)
        if self.clearing_policy == common.ClearingPolicy.ON_INITIALISE:
            self.matching_result = None
        self.check_attr = operator.attrgetter(self.variable_name)

    def update(self):
        """
        Check for existence, or the appropriate match on the expected value.

        Returns:
             :class:`~py_trees.common.Status`: :data:`~py_trees.common.Status.FAILURE` if not matched, :data:`~py_trees.common.Status.SUCCESS` otherwise.
        """
        self.logger.debug("%s.update()" % self.__class__.__name__)
        if self.matching_result is not None:
            return self.matching_result

        # existence failure check
        try:
            value = self.check_attr(self.blackboard)
            # if existence check required only
            if self.expected_value is None:
                self.feedback_message = "'%s' exists on the blackboard (as required)" % self.variable_name
                result = common.Status.SUCCESS
            # expected value matching
            else:
                success = self.comparison_operator(value, self.expected_value)
                if success:
                    self.feedback_message = "'%s' comparison succeeded [v: %s][e: %s]" % (self.variable_name, value, self.expected_value)
                    result = common.Status.SUCCESS
                else:
                    self.feedback_message = "'%s' comparison failed [v: %s][e: %s]" % (self.variable_name, value, self.expected_value)
                    result = common.Status.RUNNING
        except AttributeError:
            self.feedback_message = 'blackboard variable {0} did not exist'.format(self.variable_name)
            result = common.Status.RUNNING

        if result == common.Status.SUCCESS and self.clearing_policy == common.ClearingPolicy.ON_SUCCESS:
            self.matching_result = None
        elif result != common.Status.RUNNING:  # will fall in here if clearing ON_INITIALISE, or NEVER
            self.matching_result = result
        return result

    def terminate(self, new_status):
        """
        Always discard the matching result if it was invalidated by a parent or
        higher priority interrupt.
        """
        self.logger.debug("%s.terminate(%s)" % (self.__class__.__name__, "%s->%s" % (self.status, new_status) if self.status != new_status else "%s" % new_status))
        if new_status == common.Status.INVALID:
            self.matching_result = None
