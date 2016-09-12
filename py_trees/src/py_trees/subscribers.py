#
# License: Yujin
#
##############################################################################
# Description
##############################################################################

"""
.. module:: subscribers
   :platform: Unix
   :synopsis: Generic ros subscriber patterns.

This module provides various behaviours that utilise common usage patterns
with ros subscribers.

.. warning::

   The subscriber for all of these classes runs in the background
   runs continuously in the background. Do not use with high frequency/large
   data topics!

If you want to connect to a high frequency/large data topic, consider
creating a different class. It would complicate the mechanics too much
to add to these and in most cases, you don't want to be flinging around
large data at high frequence in behaviours anyway - it's a decisional tool,
not a computational tool.

"""

##############################################################################
# Imports
##############################################################################

import copy
import operator
import rospy
import threading

from . import behaviour
from . import blackboard
from . import common
from . import logging

##############################################################################
# Behaviours
##############################################################################


class SubscriberHandler(behaviour.Behaviour):
    """
    Not intended for direct use, as this just absorbs the mechanics of setting up
    a subscriber for inheritance by user-defined behaviour classes. There are several
    options for the mechanism of clearing the data so that a new result can be processed.

    **Always Look for New Data**

    This will clear any currently stored data upon entry into the behaviour (i.e. when
    :py:meth:`initialise` is called). This is useful for a behaviour in a sequence
    that is looking to start fresh as it is about to tick and be
    in a :py:data:`~py_trees.common.Status.RUNNING` state
    until new data arrives.

    **Look for New Data only after SUCCESS**

    This will clear any currently stored data as soon as the behaviour returns
    :py:data:`~py_trees.common.Status.SUCCESS`. This is useful for catching new
    triggers/events from things like buttons where you don't want to miss things
    even though you may not actually be ticking.

    **Use the Latest Data**

    Even if this data was received before entry into the behaviour. In this case
    :py:meth:`initialise` does not do anything with the currently stored data. Useful
    as a blocking behaviour to wait on some some topic having been initialised with
    some data (e.g. CameraInfo).
    """
    def __init__(self,
                 name="Subscriber Handler",
                 topic_name="/foo",
                 topic_type=None,
                 clearing_policy=common.ClearingPolicy.ON_INITIALISE
                 ):
        """
        :param str name: name of the behaviour
        :param str topic_name: name of the topic to connect to
        :param obj topic_type: class of the message type (e.g. std_msgs.String)
        :param clearing_policy: when to clear the data, see :py:class:`~py_trees.common.ClearingPolicy`
        """
        super(SubscriberHandler, self).__init__(name)
        self.topic_name = topic_name
        self.topic_type = topic_type
        self.msg = None
        self.subscriber = None
        self.data_guard = threading.Lock()
        self.clearing_policy = clearing_policy

    def setup(self, timeout):
        # ros doesn't care if it is init'd or not for subscriber construction, but
        # good to have here anyway so initialisation can occur before the callback is connected
        self.subscriber = rospy.Subscriber(self.topic_name, self.topic_type, self._callback, queue_size=5)
        return True

    def initialise(self):
        """
        Clears the internally stored message ready for a new run
        if ``old_data_is_valid`` wasn't set.
        """
        self.logger.debug("  %s [SubscriberHandler::initialise()]" % self.name)
        with self.data_guard:
            if self.clearing_policy == common.ClearingPolicy.ON_INITIALISE:
                self.msg = None

    def _callback(self, msg):
        """
        Subscriber callback, just stored the message.
        """
        with self.data_guard:
            self.msg = msg
            # else ignore it


class CheckSubscriberVariable(SubscriberHandler):
    """
    Check a subscriber to see if it has a specific variable
    and optionally whether that variable has a specific value.

    **Usage Patterns**

    *Sequence Guard*: RUNNING until there is a successful comparison

    - fail_if_no_data=False
    - fail_if_bad_comparison=False

    *Selector Priority Chooser*: FAILURE until there is a successful comparison

    - fail_if_no_data=True
    - fail_if_bad_comparison=True
    """
    def __init__(self,
                 name="Check Subscriber Variable",
                 topic_name="/foo",
                 topic_type=None,
                 variable_name="bar",
                 expected_value=None,
                 fail_if_no_data=False,
                 fail_if_bad_comparison=False,
                 comparison_operator=operator.eq,
                 clearing_policy=common.ClearingPolicy.ON_INITIALISE
                 ):
        """
        :param str name: name of the behaviour
        :param str topic_name: name of the topic to connect to
        :param obj topic_type: class of the message type (e.g. std_msgs.String)
        :param str variable_name: name of the variable to check
        :param obj expected_value: expected value of the variable
        :param bool fail_if_no_data: FAILURE instead of RUNNING if there is no data yet
        :param bool fail_if_bad_comparison: FAILURE instead of RUNNING if comparison failed
        :param function comparison_operator: one from the python `operator module`_
        :param clearing_policy: when to clear the data, see :py:class:`~py_trees.common.ClearingPolicy`

        .. _operator module: https://docs.python.org/2/library/operator.html
        """
        super(CheckSubscriberVariable, self).__init__(
            name,
            topic_name=topic_name,
            topic_type=topic_type,
            clearing_policy=clearing_policy,
        )
        self.variable_name = variable_name
        self.expected_value = expected_value
        self.comparison_operator = comparison_operator
        self.fail_if_no_data = fail_if_no_data
        self.fail_if_bad_comparison = fail_if_bad_comparison

    def update(self):
        """
        Handles all the logic for determining what result should go back.

        :returns: :py:data:`~py_trees.common.Status.FAILURE`, :py:data:`~py_trees.common.Status.RUNNING`, or :py:data:`~py_trees.common.Status.SUCCESS` depending on the logic.
        """
        self.logger.debug("  %s [CheckSubscriberVariable::update()]" % self.name)
        with self.data_guard:
            msg = copy.copy(self.msg)
        if msg is None:
            self.feedback_message = "have not yet received any messages"
            return common.Status.FAILURE if self.fail_if_no_data else common.Status.RUNNING

        check_attr = operator.attrgetter(self.variable_name)
        try:
            value = check_attr(msg)
        except AttributeError:
            rospy.logerr("Behaviours [%s" % self.name + "]: variable name not found [%s]" % self.variable_name)
            print("%s" % msg)
            with self.data_guard:
                self.feedback_message = "variable name not found [%s]" % self.variable_name
                return common.Status.FAILURE

        success = self.comparison_operator(value, self.expected_value)

        if success:
            self.feedback_message = "'%s' comparison succeeded [v: %s][e: %s]" % (self.variable_name, value, self.expected_value)
            if self.clearing_policy == common.ClearingPolicy.ON_SUCCESS:
                with self.data_guard:
                    self.msg = None
            return common.Status.SUCCESS
        else:
            self.feedback_message = "'%s' comparison failed [v: %s][e: %s]" % (self.variable_name, value, self.expected_value)
            return common.Status.FAILURE if self.fail_if_bad_comparison else common.Status.RUNNING


class WaitForSubscriberData(SubscriberHandler):
    """
    Waits for a subscriber's callback to be triggered. This doesn't care about
    the actual data, just whether it arrived or not, so is useful for catching
    triggers (std_msgs/Empty messages) or blocking (in the behaviour sense)
    until some data has arrived (e.g. camera configuration). There are
    two use cases:

    **Usage Patterns**

    *Got Something, Sometime*

    * clearing_policy == :py:data:`~py_trees.common.ClearingPolicy.NEVER`

    Don't care when data arrived, just that it arrived. This could be for something
    like a map topic, or a configuration that you need to block. Once it returns SUCCESS,
    it will always return SUCCESS.

    *Wating for the Next Thing, Starting From Now*

    * clearing_policy == :py:data:`~py_trees.common.ClearingPolicy.ON_INTIALISE`

    Useful as a gaurd at the start of a sequence, that is waiting for an event to
    trigger after the sequence has started (i.e. been initialised).

    *Wating for the Next Thing, Starting from the Last*

    * clearing_policy == :py:data:`~py_trees.common.ClearingPolicy.ON_SUCCESS`

    Useful as a guard watching for an event that could have come in anytime, but for
    which we do with to reset (and subsequently look for the next event). e.g. button events.
    """
    def __init__(self,
                 name="Wait For Subscriber",
                 topic_name="chatter",
                 topic_type=None,
                 clearing_policy=common.ClearingPolicy.ON_INITIALISE
                 ):
        """
        :param str name: name of the behaviour
        :param str topic_name: name of the topic to connect to
        :param obj topic_type: class of the message type (e.g. std_msgs.String)
        :param clearing_policy: when to clear the data, see :py:class:`~py_trees.common.ClearingPolicy`
        """
        super(WaitForSubscriberData, self).__init__(
            name,
            topic_name=topic_name,
            topic_type=topic_type,
            clearing_policy=clearing_policy
        )

    def update(self):
        """
        :returns: :py:data:`~py_trees.common.Status.RUNNING` (no data) or :py:data:`~py_trees.common.Status.SUCCESS`
        """
        self.logger.debug("  %s [WaitForSubscriberData::update()]" % self.name)
        with self.data_guard:
            if self.msg is None:
                self.feedback_message = "no message received yet"
                return common.Status.RUNNING
            else:
                self.feedback_message = "got incoming"
                if self.clearing_policy == common.ClearingPolicy.ON_SUCCESS:
                    self.msg = None
                return common.Status.SUCCESS


class ToBlackboard(SubscriberHandler):
    """
    Saves the latest message to the blackboard and immediately returns success.
    If no data has yet been received, this behaviour blocks (i.e. returns
    RUNNING).

    Typically this will save the entire message, however sub fields can be
    designated, in which case they will write to the specified keys.
    """
    def __init__(self,
                 name="ToBlackboard",
                 topic_name="chatter",
                 topic_type=None,
                 blackboard_variables={"chatter": None},
                 initialise_variables={},
                 clearing_policy=common.ClearingPolicy.ON_INITIALISE
                 ):
        """
        :param str name: name of the behaviour
        :param str topic_name: name of the topic to connect to
        :param obj topic_type: class of the message type (e.g. std_msgs.String)
        :param dict blackboard_variables: blackboard variable string or dict {names (keys) - message subfields (values)}
        :param clearing_policy: when to clear the data, see :py:class:`~py_trees.common.ClearingPolicy`

        If a dict is used to designate the blackboard variables, then a value of None will force the entire
        message to be saved to the string identified by the key.

        .. code-block:: python

           blackboard_variables={"pose_with_covariance_stamped": None, "pose": "pose.pose"}
        """
        super(ToBlackboard, self).__init__(
            name,
            topic_name=topic_name,
            topic_type=topic_type,
            clearing_policy=clearing_policy
        )
        self.logger = logging.get_logger("%s" % self.name)
        self.blackboard = blackboard.Blackboard()
        if isinstance(blackboard_variables, basestring):
            self.blackboard_variable_mapping = {blackboard_variables: None}
            if not isinstance(initialise_variables, dict):
                self.blackboard_initial_variable_mapping = {blackboard_variables: initialise_variables}
            else:
                self.blackboard_initial_variable_mapping = initialise_variables
        elif not isinstance(blackboard_variables, dict):
            self.logger.error("blackboard_variables is not a dict, please rectify")
            self.blackboard_variable_mapping = {}
            self.blackboard_initial_variable_mapping = {}
        else:
            self.blackboard_variable_mapping = blackboard_variables
            self.blackboard_initial_variable_mapping = initialise_variables
        # initialise the variables
        for name, value in self.blackboard_initial_variable_mapping.iteritems():
            if not self.blackboard.set(name, value):
                # do we actually want to log an error?
                self.logger.error("tried to initialise an already initialised blackboard variable '{0}', check that you do not have a conflict with another behaviour [{1}]".format(name, self.name))

    def setup(self, timeout):
        return super(ToBlackboard, self).setup(timeout)

    def update(self):
        """
        Writes the data (if available) to the blackboard.

        :returns: :py:data:`~py_trees.common.Status.RUNNING` (no data) or :py:data:`~py_trees.common.Status.SUCCESS`
        """
        with self.data_guard:
            if self.msg is None:
                self.feedback_message = "no message received yet"
                return common.Status.RUNNING
            else:
                for k, v in self.blackboard_variable_mapping.iteritems():
                    if v is None:
                        self.blackboard.set(k, self.msg, overwrite=True)
                    else:
                        fields = v.split(".")
                        value = copy.copy(self.msg)
                        for field in fields:
                            value = getattr(value, field)
                            self.blackboard.set(k, value, overwrite=True)
                self.feedback_message = "saved incoming message"
                return common.Status.SUCCESS


##############################################################################
# Deprecating
##############################################################################

# class CheckSubscriberVariable2(behaviour.Behaviour):
#     """
#     Check a subscriber to see if it has a specific variable
#     and optionally whether that variable has a specific value.
#
#     .. todo:: base this off the subscriber handler.
#
#     """
#     def __init__(self,
#                  name="Check Subscriber Variable",
#                  topic_name="/foo",
#                  topic_type=None,
#                  variable_name="bar",
#                  expected_value=None,
#                  fail_if_no_data=False,
#                  fail_if_bad_comparison=False,
#                  comparison_operator=operator.eq,
#                  monitor_continuously=False
#                  ):
#         """
#         :param str name: name of the behaviour
#         :param str topic_name: name of the topic to connect to
#         :param obj topic_type: class of the message type (e.g. std_msgs.String)
#         :param str variable_name: name of the variable to check
#         :param obj expected_value: expected value of the variable
#         :param bool fail_if_no_data: return FAILURE instead of RUNNING if there is no data yet
#         :param bool fail_if_bad_comparison: return FAILURE instead of RUNNING if there is data, but failed comparison
#         :param function comparison_operator: one of the comparison operators from the python operator module
#         :param bool monitor_continuously: setup subscriber immediately and continuously monitor the data
#
#         Usage Patterns:
#
#         As a guard at the start of a sequence (RUNNING until there is a successful comparison):
#
#         - fail_if_no_data=False
#         - fail_if_bad_comparison=False
#         - monitor_contiuously=False
#
#         As a priority chooser in a selector (FAILURE until there is a successful comparison)
#
#         - fail_if_no_data=True
#         - fail_if_bad_comparison=True
#         - monitor_contiuously=True
#
#         Note : if you set fail_if_no_data, then you should use the monitor_continuously flag as setting
#         the subscriber every time you enter the cell is not likely to give it enough time to collect data (unless
#         it is a latched topic).
#
#         .. seealso:: https://docs.python.org/2/library/operator.html
#         """
#         super(CheckSubscriberVariable, self).__init__(name)
#         self.logger.debug("  %s [CheckSubscriberVariable::initialise()]" % self.name)
#         self.topic_name = topic_name
#         self.topic_type = topic_type
#         self.variable_name = variable_name
#         self.expected_value = expected_value
#         self.comparison_operator = comparison_operator
#         self.subscriber = None
#         self.fail_if_no_data = fail_if_no_data
#         self.fail_if_bad_comparison = fail_if_bad_comparison
#         # Use cached_xxx variables to generate a status.feedback in the callback independent
#         # of the behaviour tick status (i.e. it runs and updates continuously). The
#         # behaviour's tick status then reflects this when required
#         self.cached_status = common.Status.INVALID
#         self.cached_feedback = ""
#         self.data_guard = threading.Lock()
#         self.monitor_continuously = monitor_continuously
#
#     def initialise(self):
#         self.logger.debug("  %s [CheckSubscriberVariable::initialise()]" % self.name)
#         if not self.monitor_continuously:
#             self.setup(None)
#
#     def terminate(self, new_status):
#         """
#         Cleanup if we're not tasked with monitoring continuously.
#
#         .. todo::
#
#            It might be that we may wish to reset variables on stop, even if monitoring
#            continously. If that turns out to be a use case, we need to implement
#            something for that.
#         """
#         self.logger.debug("  %s [CheckSubscriberVariable::terminate()]" % self.name)
#         if not self.monitor_continuously:
#             with self.data_guard:
#                 self.cached_status = new_status
#                 self.cached_feedback = ""
#                 if self.subscriber is not None:
#                     self.subscriber.unregister()
#                     self.subscriber = None
#
#     def setup(self, unused_timeout):
#         self.logger.debug("  %s [CheckSubscriberVariable::setup()]" % self.name)
#         """
#         This will get called early if it was flagged in the constructor. This is useful
#         if you want to continuously monitor and just reflect the result as needed.
#         The converse case is when you only want to start monitoring as soon as the behaviour
#         begins running (i.e. on initialise).
#         """
#         with self.data_guard:
#             self.cached_status = common.Status.FAILURE if self.fail_if_no_data else common.Status.RUNNING
#             self.cached_feedback = "have not yet received any messages"
#         # if it's a latched publisher, this will immediately trigger a callback, inside the constructor!
#         # so make sure we aren't in a lock ourselves, otherwise we get a deadlock
#         self.subscriber = rospy.Subscriber(self.topic_name, self.topic_type, self._callback)
#         return True
#
#     def _callback(self, msg):
#         """
#         Checks if the message has the expected variable name and then checks for the expected value.
#         """
#         if not hasattr(msg, self.variable_name):
#             rospy.logerr("Behaviours [%s" % self.name + "]: expected variable name in message not found [%s]" % self.variable_name)
#             with self.data_guard:
#                 self.cached_feedback = "expected variable name in message not found [%s]" % self.variable_name
#                 self.cached_status = common.Status.FAILURE
#             return
#         value = getattr(msg, self.variable_name)
#         success = self.comparison_operator(value, self.expected_value)
#
#         with self.data_guard:
#             if success:
#                 self.cached_feedback = "'%s' comparison succeeded [v: %s][e: %s]" % (self.variable_name, value, self.expected_value)
#                 self.cached_status = common.Status.SUCCESS
#             else:
#                 self.cached_feedback = "'%s' comparison failed [v: %s][e: %s]" % (self.variable_name, value, self.expected_value)
#                 self.cached_status = common.Status.FAILURE if self.fail_if_bad_comparison else common.Status.RUNNING
#
#     def update(self):
#         self.logger.debug("  %s [CheckSubscriberVariable::update()]" % self.name)
#         if self.subscriber is None:
#             self.feedback_message = "no subscriber, did you call setup() on your tree?"
#             return common.Status.FAILURE
#         with self.data_guard:
#             self.feedback_message = copy.copy(self.cached_feedback)
#             status = copy.copy(self.cached_status)
#         return status
