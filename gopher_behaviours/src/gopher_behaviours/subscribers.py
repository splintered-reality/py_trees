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
import py_trees
import rospy
import threading

##############################################################################
# Behaviour
##############################################################################


class CheckSubscriberVariable(py_trees.Behaviour):
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
                 invert=False,
                 monitor_continuously=False
                 ):
        """
        :param str name: name of the behaviour
        :param str topic_name: name of the topic to connect to
        :param obj topic_type: class of the message type (e.g. std_msgs.String)
        :param str variable_name: name of the variable to check
        :param obj expected_value: expected value of the variable
        :param bool invert: if true, check that the value of the variable is != expected_value, rather than ==
        :param bool monitor_continuously: setup subscriber immediately and continuously monitor the data
        """
        super(CheckSubscriberVariable, self).__init__(name)
        self.topic_name = topic_name
        self.topic_type = topic_type
        self.variable_name = variable_name
        self.expected_value = expected_value
        self.invert = invert
        self.subscriber = None
        # Use cached_xxx variables to generate a status.feedback in the callback independent
        # of the behaviour tick status (i.e. it runs and updates continuously). The
        # behaviour's tick status then reflects this when required
        self.cached_status = py_trees.Status.INVALID
        self.cached_feedback = ""
        self.data_guard = threading.Lock()
        self.monitor_continuously = monitor_continuously
        if self.monitor_continuously:
            self._setup()

    def initialise(self):
        if not self.monitor_continuously:
            self._setup()

    def stop(self, new_status=py_trees.Status.INVALID):
        """
        Cleanup if we're not tasked with monitoring continuously.

        .. todo::

           It might be that we may wish to reset variables on stop, even if monitoring
           continously. If that turns out to be a use case, we need to implement
           something for that.
        """
        if not self.monitor_continuously:
            with self.data_guard:
                self.cached_status = new_status
                self.cached_feedback = ""
                if self.subscriber is not None:
                    self.subscriber.unregister()
                    self.subscriber = None

    def _setup(self):
        """
        This will get called early if it was flagged in the constructor. This is useful
        if you want to continuously monitor and just reflect the result as needed.
        The converse case is when you only want to start monitoring as soon as the behaviour
        begins running (i.e. on initialise).
        """
        with self.data_guard:
            self.cached_status = py_trees.Status.RUNNING
            self.cached_feedback = "have not yet received any messages"
            self.subscriber = rospy.Subscriber(self.topic_name, self.topic_type, self._callback)

    def _callback(self, msg):
        """
        Checks if the message has the expected variable name and then checks for the expected value.
        """
        if not hasattr(msg, self.variable_name):
            rospy.logerr("Behaviours [%s" % self.name + "]: expected variable name in message not found [%s]" % self.variable_name)
            with self.data_guard:
                self.cached_feedback = "expected variable name in message not found [%s]" % self.variable_name
                self.cached_status = py_trees.Status.FAILURE
            return
        value = getattr(msg, self.variable_name)
        matched_expected = (value == self.expected_value)

        with self.data_guard:
            if not self.invert:
                if matched_expected:
                    self.cached_feedback = "'%s' matched expected value (as required) [v: %s][e: %s]" % (self.variable_name, value, self.expected_value)
                    self.cached_status = py_trees.Status.SUCCESS
                else:
                    self.cached_feedback = "'%s' did not match expected value (required otherwise) [v: %s][e: %s]" % (self.variable_name, value, self.expected_value)
                    self.cached_status = py_trees.Status.FAILURE
            else:
                result = not matched_expected
                if result:
                    self.cached_feedback = "'%s' did not match expected value (as required) [v: %s][e: %s]" % (self.variable_name, value, self.expected_value)
                    self.cached_status = py_trees.Status.SUCCESS
                else:
                    self.cached_feedback = "'%s' matched expected value (required otherwise) [v: %s][e: %s]" % (self.variable_name, value, self.expected_value)
                    self.cached_status = py_trees.Status.FAILURE

    def update(self):
        with self.data_guard:
            self.feedback_message = copy.copy(self.cached_feedback)
            status = copy.copy(self.cached_status)
        return status


class SubscriberHandler(py_trees.Behaviour):
    """
    Not intended for direct use, as this just absorbs the mechanics of setting up
    a subscriber for use by children who inherit and implement their own custom
    update function.
    """
    def __init__(self,
                 name="Subscriber Handler",
                 topic_name="/foo",
                 topic_type=None,
                 monitor_continuously=False
                 ):
        """
        :param str name: name of the behaviour
        :param str topic_name: name of the topic to connect to
        :param obj topic_type: class of the message type (e.g. std_msgs.String)
        :param bool monitor_continuously: setup subscriber immediately and continuously monitor the data
        """
        super(CheckSubscriberVariable, self).__init__(name)
        self.topic_name = topic_name
        self.topic_type = topic_type
        self.msg = None
        self.subscriber = None
        self.data_guard = threading.Lock()
        self.monitor_continuously = monitor_continuously
        if self.monitor_continuously:
            self._setup()

    def initialise(self):
        if not self.monitor_continuously:
            self._setup()

    def stop(self, new_status=py_trees.Status.INVALID):
        if not self.monitor_continuously:
            with self.data_guard:
                self.msg = None
                if self.subscriber is not None:
                    self.subscriber.unregister()
                    self.subscriber = None

    def _setup(self):
        """
        This will get called early if it was flagged in the constructor. This is useful
        if you want to continuously monitor and just reflect the result as needed.
        The converse case is when you only want to start monitoring as soon as the behaviour
        begins running (i.e. on initialise).
        """
        with self.data_guard:
            self.subscriber = rospy.Subscriber(self.topic_name, self.topic_type, self._callback)

    def _callback(self, msg):
        """
        Simple stores the message for children of this class to implement a custom update function.
        """
        with self.data_guard:
            self.msg = msg
