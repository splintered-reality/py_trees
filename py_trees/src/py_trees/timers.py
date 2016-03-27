#
# License: Yujin
#
##############################################################################
# Description
##############################################################################

"""
.. module:: time
   :platform: Unix
   :synopsis: Time related behaviours.

Oh my spaghettified magnificence,
Bless my noggin with a tickle from your noodly appendages!

----

"""

##############################################################################
# Imports
##############################################################################

import time

from . import behaviour
from . import behaviours
from . import common
from . import composites
from . import meta

##############################################################################
# Behaviours
##############################################################################


class Pause(behaviour.Behaviour):
    """
    Does nothing until the specified timeout is reached, then returns SUCCESS.
    If the duration is 0.0, then this behaviour blocks indefinitely.
    """
    def __init__(self, name="Pause", duration=1.0):
        """
        Prepare the behaviour

        :param string name: this behaviour's name
        :param float duration: the amount of time to pause for; set to zero to pause indefinitely
        """
        super(Pause, self).__init__(name)
        self.duration = duration
        self.start_time = None
        self.finish_time = None

    def initialise(self):
        self.logger.debug("  %s [Pause::initialise()]" % self.name)
        self.start_time = time.time()
        self.finish_time = self.start_time + self.duration

    def update(self):
        self.logger.debug("  %s [Pause::update()]" % self.name)
        if self.duration == 0.0:
            return common.Status.RUNNING
        else:
            return common.Status.SUCCESS if time.time() > self.finish_time else common.Status.RUNNING


class Timer(behaviour.Behaviour):
    """
    Simple timer class that is :py:data:`~py_trees.common.Status.RUNNING` until the timer
    runs out, at which point it is :py:data:`~py_trees.common.Status.SUCCESS`.

    Use the :py:func:`~py_trees.meta.running_is_failure` if you want it to return
    :py:data:`~py_trees.common.Status.FAILURE` until the timer runs out.

    The timer gets reset either upon entry (:py:meth:`~py_trees.behaviour.Behaviour.initialise`)
    if it hasn't already been set and gets cleared when it either runs out, or the behaviour is
    interrupted (i.e. switches to :py:data:`~py_trees.common.Status.SUCCESS` or
    :py:data:`~py_trees.common.Status.INVALID`).
    """
    def __init__(self, name="Timer", duration=5.0):
        """
        :param float duration:
        """
        super(Timer, self).__init__(name)
        self.duration = duration
        self.finish_time = None

    def initialise(self):
        self.logger.debug("  %s [Timer::initialise()]" % self.name)
        if self.finish_time is None:
            self.finish_time = time.time() + self.duration

    def update(self):
        self.logger.debug("  %s [Timer::update()]" % self.name)
        current_time = time.time()
        if current_time > self.finish_time:
            self.feedback_message = "timer ran out (finished)"
            return common.Status.SUCCESS
        else:
            self.feedback_message = "still running"  # don't want spammy messages (%s)" % (self.finish_time - current_time)
            return common.Status.RUNNING

    def terminate(self, new_status):
        self.logger.debug("  %s [Timer::terminate()][%s->%s]" % (self.name, self.status, new_status))
        # clear the time if finishing with SUCCESS or in the case of an interruption from INVALID
        if new_status == common.Status.SUCCESS or new_status == common.Status.INVALID:
            self.finish_time = None


def create_timeout_subtree(behaviour, timeout):
    """
    :param Behaviour behaviour: behaviour to keep retrying until the timeout is reached.
    :param float timeout:
    """
    timeout_selector = composites.Selector("Timeout")
    timer = Timer(name="Timer", duration=timeout)
    failing_timer = meta.running_is_failure(timer)

    timeout_selector.add_child(failing_timer)
    timeout_selector.add_child(behaviour)
    timeout_selector.add_child(behaviours.Running(name="Retry"))
    return timeout_selector

# @meta.inverter
# class Timeout(Pause):
#     pass
