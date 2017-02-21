#!/usr/bin/env python
#
# License: BSD
#   https://raw.githubusercontent.com/stonier/py_trees/devel/LICENSE
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

from .common import Status
from .behaviour import Behaviour
from . import composites
from . import meta

##############################################################################
# Function Behaviours
##############################################################################


def success(self):
    self.logger.debug("%s.update()" % self.__class__.__name__)
    self.feedback_message = "success"
    return Status.SUCCESS


def failure(self):
    self.logger.debug("%s.update()" % self.__class__.__name__)
    self.feedback_message = "failure"
    return Status.FAILURE


def running(self):
    self.logger.debug("%s.update()" % self.__class__.__name__)
    self.feedback_message = "running"
    return Status.RUNNING


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

##############################################################################
# Standalone Behaviours
##############################################################################


class Periodic(Behaviour):
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
    This behaviour updates it's status with :data:`~py_trees.common.Status.SUCCESS`
    once every N ticks, :data:`~py_trees.common.Status.FAILURE` otherwise.

    Args:
        name (:obj:`str`): name of the behaviour
        n (:obj:`int`): trigger success on every n'th tick

    .. tip::
       Use with decorators to change the status value as desired, e.g.
       :meth:`py_trees.meta.failure_is_running`
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
            return Status.SUCCESS
        else:
            self.feedback_message = "not yet"
            return Status.FAILURE


class Count(Behaviour):
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
    def __init__(self, name="Count", fail_until=3, running_until=5, success_until=6, reset=True, *args, **kwargs):
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
# Composite Behaviours
##############################################################################

@meta.oneshot
class OneshotSequence(composites.Sequence):
    """
    A sequence with a oneshot decorator applied to it.
    """
    pass
