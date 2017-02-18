#!/usr/bin/env python
#
# License: BSD
#   https://raw.githubusercontent.com/stonier/py_trees_suite/devel/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
.. module:: behaviours
   :platform: Unix
   :synopsis: Behaviour templates for use in behaviour trees.

This module defines the interface for behaviours to be used in py_trees.

----

"""

##############################################################################
# Imports
##############################################################################

import rospy
from . import logging
from .common import Status
from .behaviour import Behaviour
from . import meta

##############################################################################
# Function Behaviours
##############################################################################


def success(self):
    self.logger.debug("%s [Success::update()]" % self.name)
    self.feedback_message = "success"
    return Status.SUCCESS


def failure(self):
    self.logger.debug("%s [Failure::update()]" % self.name)
    self.feedback_message = "failure"
    return Status.FAILURE


def running(self):
    self.logger.debug("%s [Running::update()]" % self.name)
    self.feedback_message = "running"
    return Status.RUNNING


Success = meta.create_behaviour_from_function(success)
Failure = meta.create_behaviour_from_function(failure)
Running = meta.create_behaviour_from_function(running)

##############################################################################
# Other Behaviours
##############################################################################


class Periodic(Behaviour):
    """
    Return running for N ticks, success for N ticks, failure for N ticks...

    Note, it does not reset the count when initialising. No special reason
    for this, just the way it was first implemented.
    """
    def __init__(self, name, n):
        super(Periodic, self).__init__(name)
        self.count = 0
        self.period = n
        self.response = Status.RUNNING

    def update(self):
        self.count += 1
        if self.count > self.period:
            if self.response == Status.FAILURE:
                self.feedback_message = "flip to running"
                self.response = Status.RUNNING
            elif self.response == Status.RUNNING:
                self.feedback_message = "flip to success"
                self.response = Status.SUCCESS
            else:
                self.feedback_message = "flip to failure"
                self.response = Status.FAILURE
            self.count = 0
        else:
            self.feedback_message = "constant"
        return self.response


class SuccessEveryN(Behaviour):
    """
    Return success once every N ticks, failure otherwise.
    """
    def __init__(self, name, n):
        super(SuccessEveryN, self).__init__(name)
        self.count = 0
        self.every_n = n

    def update(self):
        self.count += 1
        self.logger.debug("%s [SuccessEveryN::update()][%s]" % (self.name, self.count))
        if self.count % self.every_n == 0:
            self.feedback_message = "now"
            return Status.SUCCESS
        else:
            self.feedback_message = "not yet"
            return Status.FAILURE


class Count(Behaviour):
    def __init__(self, name="Count", fail_until=3, running_until=5, success_until=6, reset=True, *args, **kwargs):
        """
        :param str name:
        :param int fail_until:
        :param int running_until:
        :param int success_until:
        :param bool reset: reset whenever invalidated (usually by a sequence reinitialising, or higher priority selection knocking it out)
        """
        super(Count, self).__init__(name, *args, **kwargs)
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
        if new_status == Status.INVALID and self.reset:
            self.count = 0
            self.number_count_resets += 1
        self.feedback_message = ""

    def update(self):
        self.number_updated += 1
        self.count += 1
        if self.count <= self.fail_until:
            self.logger.debug("%s.update()[%s: failure]" % (self.__class__.__name__, self.count))
            self.feedback_message = "failing"
            return Status.FAILURE
        elif self.count <= self.running_until:
            self.logger.debug("%s.update()[%s: running]" % (self.__class__.__name__, self.count))
            self.feedback_message = "running"
            return Status.RUNNING
        elif self.count <= self.success_until:
            self.logger.debug("%s.update()[%s: success]" % (self.__class__.__name__, self.count))
            self.feedback_message = "success"
            return Status.SUCCESS
        else:
            self.logger.debug("%s.update()[%s: failure]" % (self.__class__.__name__, self.count))
            self.feedback_message = "failing forever more"
            return Status.FAILURE

    def __repr__(self):
        s = "%s\n" % self.name
        s += "  Status : %s\n" % self.status
        s += "  Count  : %s\n" % self.count
        s += "  Resets : %s\n" % self.number_count_resets
        s += "  Updates: %s\n" % self.number_updated
        return s


class Condition(Behaviour):
    """
    This behaviour will block (i.e. maintain a RUNNING status) until its child behaviour
    returns the specified status. This behaviour can be used as a sort of loop on a
    specific behaviour. For example, we implement a behaviour which checks the battery
    level and returns success or failure depending on the threshold specified.
    Somewhere else, we want to wait until the battery level reaches a specific
    level. To do this, we could create a separate wait behaviour for the battery
    level, which would be running until the battery level reaches a specific
    level, perhaps with a timeout. That behaviour would look almost identical to
    the check behaviour. Instead, we put the check behaviour as the child of
    this behaviour, and it will do the waiting without requiring an additional
    behaviour.

    :param child: the child behaviour to run
    :param succeed_status: the status that the child needs to have in order for
        this behaviour to return success
    :param timeout: if the behaviour is not in the succeed_status by this number
        of seconds after this behaviour starts, the behaviour fails

    """
    def __init__(self, name, child, succeed_status, timeout=None):
        super(Condition, self).__init__(name)
        self.duration = rospy.Duration(timeout) if timeout else None
        self.timeout = None
        self.children = [child]
        self.succeed_status = succeed_status

    def initialise(self):
        self.timeout = rospy.Time.now() + self.duration if self.duration else None

    def update(self):
        return Status.RUNNING

    def tick(self):
        for unused_t in super(Condition, self).tick():
            pass

        if self.timeout and self.timeout < rospy.Time.now():
            self.feedback_message = 'timed out'
            self.stop(Status.FAILURE)
            yield self

        # tick the child to run it and its children, if any
        for node in self.children[0].tick():
            yield node
            # if the child status is the status we were waiting for, succeed
            if node is self.children[0] and node.status == self.succeed_status:
                self.feedback_message = 'child node entered the desired state'
                self.stop(Status.SUCCESS)
                break
        yield self
