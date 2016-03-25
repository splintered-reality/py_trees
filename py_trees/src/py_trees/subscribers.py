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

Oh my spaghettified magnificence,
Bless my noggin with a tickle from your noodly appendages!
----

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

##############################################################################
# Behaviour
##############################################################################


class CheckSubscriberVariable(behaviour.Behaviour):
    """
    Check a subscriber to see if it has a specific variable
    and optionally whether that variable has a specific value.

    .. todo:: base this off the subscriber handler.

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
                 monitor_continuously=False
                 ):
        """
        :param str name: name of the behaviour
        :param str topic_name: name of the topic to connect to
        :param obj topic_type: class of the message type (e.g. std_msgs.String)
        :param str variable_name: name of the variable to check
        :param obj expected_value: expected value of the variable
        :param bool fail_if_no_data: return FAILURE instead of RUNNING if there is no data yet
        :param bool fail_if_bad_comparison: return FAILURE instead of RUNNING if there is data, but failed comparison
        :param function comparison_operator: one of the comparison operators from the python operator module
        :param bool monitor_continuously: setup subscriber immediately and continuously monitor the data

        Usage Patterns:

        As a guard at the start of a sequence (RUNNING until there is a successful comparison):

        - fail_if_no_data=False
        - fail_if_bad_comparison=False
        - monitor_contiuously=False

        As a priority chooser in a selector (FAILURE until there is a successful comparison)

        - fail_if_no_data=True
        - fail_if_bad_comparison=True
        - monitor_contiuously=True

        Note : if you set fail_if_no_data, then you should use the monitor_continuously flag as setting
        the subscriber every time you enter the cell is not likely to give it enough time to collect data (unless
        it is a latched topic).

        .. seealso:: https://docs.python.org/2/library/operator.html
        """
        super(CheckSubscriberVariable, self).__init__(name)
        self.logger.debug("  %s [CheckSubscriberVariable::initialise()]" % self.name)
        self.topic_name = topic_name
        self.topic_type = topic_type
        self.variable_name = variable_name
        self.expected_value = expected_value
        self.comparison_operator = comparison_operator
        self.subscriber = None
        self.fail_if_no_data = fail_if_no_data
        self.fail_if_bad_comparison = fail_if_bad_comparison
        # Use cached_xxx variables to generate a status.feedback in the callback independent
        # of the behaviour tick status (i.e. it runs and updates continuously). The
        # behaviour's tick status then reflects this when required
        self.cached_status = common.Status.INVALID
        self.cached_feedback = ""
        self.data_guard = threading.Lock()
        self.monitor_continuously = monitor_continuously

    def initialise(self):
        self.logger.debug("  %s [CheckSubscriberVariable::initialise()]" % self.name)
        if not self.monitor_continuously:
            self.setup(None)

    def terminate(self, new_status):
        """
        Cleanup if we're not tasked with monitoring continuously.

        .. todo::

           It might be that we may wish to reset variables on stop, even if monitoring
           continously. If that turns out to be a use case, we need to implement
           something for that.
        """
        self.logger.debug("  %s [CheckSubscriberVariable::terminate()]" % self.name)
        if not self.monitor_continuously:
            with self.data_guard:
                self.cached_status = new_status
                self.cached_feedback = ""
                if self.subscriber is not None:
                    self.subscriber.unregister()
                    self.subscriber = None

    def setup(self, unused_timeout):
        self.logger.debug("  %s [CheckSubscriberVariable::setup()]" % self.name)
        """
        This will get called early if it was flagged in the constructor. This is useful
        if you want to continuously monitor and just reflect the result as needed.
        The converse case is when you only want to start monitoring as soon as the behaviour
        begins running (i.e. on initialise).
        """
        with self.data_guard:
            self.cached_status = common.Status.FAILURE if self.fail_if_no_data else common.Status.RUNNING
            self.cached_feedback = "have not yet received any messages"
        # if it's a latched publisher, this will immediately trigger a callback, inside the constructor!
        # so make sure we aren't in a lock ourselves, otherwise we get a deadlock
        self.subscriber = rospy.Subscriber(self.topic_name, self.topic_type, self._callback)
        return True

    def _callback(self, msg):
        """
        Checks if the message has the expected variable name and then checks for the expected value.
        """
        if not hasattr(msg, self.variable_name):
            rospy.logerr("Behaviours [%s" % self.name + "]: expected variable name in message not found [%s]" % self.variable_name)
            with self.data_guard:
                self.cached_feedback = "expected variable name in message not found [%s]" % self.variable_name
                self.cached_status = common.Status.FAILURE
            return
        value = getattr(msg, self.variable_name)
        success = self.comparison_operator(value, self.expected_value)

        with self.data_guard:
            if success:
                self.cached_feedback = "'%s' comparison succeeded [v: %s][e: %s]" % (self.variable_name, value, self.expected_value)
                self.cached_status = common.Status.SUCCESS
            else:
                self.cached_feedback = "'%s' comparison failed [v: %s][e: %s]" % (self.variable_name, value, self.expected_value)
                self.cached_status = common.Status.FAILURE if self.fail_if_bad_comparison else common.Status.RUNNING

    def update(self):
        self.logger.debug("  %s [CheckSubscriberVariable::update()]" % self.name)
        if self.subscriber is None:
            self.feedback_message = "no subscriber, did you call setup() on your tree?"
            return common.Status.FAILURE
        with self.data_guard:
            self.feedback_message = copy.copy(self.cached_feedback)
            status = copy.copy(self.cached_status)
        return status


class SubscriberHandler(behaviour.Behaviour):
    """
    Not intended for direct use, as this just absorbs the mechanics of setting up
    a subscriber for use by children who inherit and implement their own custom
    update function.
    """
    def __init__(self,
                 name="Subscriber Handler",
                 topic_name="/foo",
                 topic_type=None,
                 monitor_continuously=False,
                 ignore_latched_messages=False
                 ):
        """
        :param str name: name of the behaviour
        :param str topic_name: name of the topic to connect to
        :param obj topic_type: class of the message type (e.g. std_msgs.String)
        :param bool monitor_continuously: setup subscriber immediately and continuously monitor the data
        :param bool ignore_latched_messages: if data comes in during the Subscriber's constructor, then throw away if this is True.
        """
        super(SubscriberHandler, self).__init__(name)
        self.topic_name = topic_name
        self.topic_type = topic_type
        self.msg = None
        self.subscriber = None
        self.data_guard = threading.Lock()
        self.latched_callback_guard = threading.Lock()
        self.monitor_continuously = monitor_continuously
        self.ignore_latched_messages = ignore_latched_messages
        if self.monitor_continuously:
            self._setup()

    def initialise(self):
        self.logger.debug("  %s [SubscriberHandler::initialise()]" % self.name)
        if not self.monitor_continuously:
            self.setup(timeout=None)

    def terminate(self, new_status):
        self.logger.debug("  %s [SubscriberHandler::terminate()][%s->%s]" % (self.name, self.status, new_status))
        if not self.monitor_continuously:
            with self.data_guard:
                self.msg = None
                if self.subscriber is not None:
                    print("Unregister")
                    self.subscriber.unregister()
                    self.subscriber = None

    def setup(self, timeout):
        """
        This is necessary if you want to continuously monitor and just reflect the result as needed.
        Note that the constructor automatically calls this (ros doesn't care if you are init'd or
        not at this point).

        The converse case is when you only want to start monitoring as soon as the behaviour
        begins running, this will happen via initialise if monitor_continuously is not True.
        """
        self.logger.debug("  %s [SubscriberHandler::setup()]" % self.name)
        # if it's a latched publisher, this will immediately trigger a callback, inside the constructor!
        # so make sure we aren't in a lock ourselves, otherwise we get a deadlock
        with self.latched_callback_guard:
            print("Setting up the subscriber on %s [%s]" % (self.topic_name, self.topic_type))
            self.subscriber = rospy.Subscriber(self.topic_name, self.topic_type, self._callback, queue_size=5)
        return True

    def _callback(self, msg):
        """
        Simple stores the message for children of this class to implement a custom update function.
        """
        print("_callback")
        with self.data_guard:
            this_is_not_a_latched_callback = self.latched_callback_guard.acquire(False)
            print("this_is_not_a_latched_callback %s" % this_is_not_a_latched_callback)
            if this_is_not_a_latched_callback or not self.ignore_latched_messages:
                print("Setting message")
                self.msg = msg
            # else ignore it


class WaitForSubscriberData(SubscriberHandler):
    """
    Waits for a subscriber's callback to be triggered. This is mostly useful
    for std_msgs.Empty topics.
    """
    def __init__(self,
                 name="Wait For Subscriber",
                 topic_name="chatter",
                 topic_type=None,
                 ignore_latched_messages=False
                 ):
        """
        :param str name: name of the behaviour
        :param str topic_name: name of the topic to connect to
        :param obj topic_type: class of the message type (e.g. std_msgs.String)
        """
        super(WaitForSubscriberData, self).__init__(
            name,
            topic_name=topic_name,
            topic_type=topic_type,
            monitor_continuously=False,
            ignore_latched_messages=ignore_latched_messages
        )

    def update(self):
        """
        RUNNING if no data yet, SUCCESS otherwise.
        """
        self.logger.debug("  %s [WaitForSubscriberData::update()]" % self.name)
        self.logger.info("  %s [WaitForSubscriberData::update()]" % self.name)
        with self.data_guard:
            if self.msg is None:
                self.feedback_message = "no message received yet"
                return common.Status.RUNNING
            else:
                self.feedback_message = "got incoming"
                return common.Status.SUCCESS


class SubscriberToBlackboard(SubscriberHandler):
    """
    Saves the latest message to the blackboard and immediately returns success.
    If no data has yet been received, this behaviour blocks (i.e. returns
    RUNNING).

   Typically this will save the entire message, however sub fields can be
    designated, in which case they will write to the specified keys.
    """
    def __init__(self,
                 name="SubscriberToBlackboard",
                 topic_name="chatter",
                 topic_type=None,
                 blackboard_variables={"chatter": None},
                 monitor_continuously=False
                 ):
        """
        :param str name: name of the behaviour
        :param str topic_name: name of the topic to connect to
        :param obj topic_type: class of the message type (e.g. std_msgs.String)
        :param dict blackboard_variables: blackboard variable string or dict {names (keys) - message subfields (values)}
        :param bool monitor_continuously: setup subscriber immediately and continuously monitor the data

        If a dict is used to designate the blackboard variables, then a value of None will force the entire
        message to be saved to the string identified by the key.

        .. code-block:: python

           blackboard_variables={"pose_with_covariance_stamped": None, "pose": "pose.pose"}
        """
        super(SubscriberToBlackboard, self).__init__(
            name,
            topic_name=topic_name,
            topic_type=topic_type,
            monitor_continuously=monitor_continuously
        )
        self.blackboard = blackboard.Blackboard()
        if isinstance(blackboard_variables, basestring):
            self.blackboard_variable_mapping = {blackboard_variables: None}
        elif not isinstance(blackboard_variables, dict):
            rospy.logerr("SubscriberToBlackboard: blackboard_variables is not a dict, please rectify [%s]" % self.name)
            self.blackboard_variable_mapping = {}
        else:
            self.blackboard_variable_mapping = blackboard_variables

    def update(self):
        """
        Writes the data to the blackboard.
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
