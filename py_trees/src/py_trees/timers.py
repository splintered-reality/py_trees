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
from . import common
from . import meta

##############################################################################
# Behaviours
##############################################################################


# class Pause(behaviour.Behaviour):
#     """
#     Does nothing until the specified timeout is reached, then returns SUCCESS.
#     If the duration is 0.0, then this behaviour blocks indefinitely.
#
#     This can also be used as a timeout with highest priority in a selector.
#     """
#     def __init__(self, name="Pause", duration=1.0):
#         """
#         Prepare the behaviour
#
#         :param string name: this behaviour's name
#         :param float duration: the amount of time to pause for; set to zero to pause indefinitely
#         """
#         super(Pause, self).__init__(name)
#         self.duration = duration
#         self.start_time = None
#         self.finish_time = None
#
#     def initialise(self):
#         self.logger.debug("  %s [Pause::initialise()]" % self.name)
#         self.start_time = time.time()
#         self.finish_time = self.start_time + self.duration
#
#     def update(self):
#         self.logger.debug("  %s [Pause::update()]" % self.name)
#         if self.duration == 0.0:
#             return common.Status.RUNNING
#         else:
#             return common.Status.SUCCESS if time.time() > self.finish_time else common.Status.RUNNING


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

    **Usage Patterns**

    *TimeOut*

    Typical use case is at the highest priority branch of a selector. As soon
    as the timeout activates, then all other branches are invalidated.

    This does mean however, that the selector should
    return :py:data:`~py_trees.common.Status.FAILURE` to signify that the actual
    non-timeout branches succeeded. A typical code block would look like:

    .. code-block:: python

       root = py_trees.meta.inverter(py_trees.composites.Selector("Foo"))
       timeout = py_trees.meta.running_is_failure(py_trees.timers.Timeout("Timeout", 10.0))
       work = py_trees.meta.inverter(py_trees.composites.Sequence("Work"))
       work_behaviour_one = my.work.behaviours.One("One")
       work_behaviour_two = my.work.behaviours.Two("Two")

       root.add_child(timeout)
       root.add_child(work)
       work.add_child(work_behaviour_one)
       work.add_child(work_behaviour_two)

    This will return:

    * `~py_trees.common.Status.RUNNING` if no timeout and the sequence is not yet finished
    * `~py_trees.common.Status.FAILURE` if timeout has been exceeded
    * `~py_trees.common.Status.SUCCESS` if the sequence has finished successfully.
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
            # do not show the time, it causes the tree to be 'changed' every tick
            # and we don't want to spam visualisations with almost meaningless updates
            self.feedback_message = "still running"  # (%s)" % (self.finish_time - current_time)
            return common.Status.RUNNING

    def terminate(self, new_status):
        self.logger.debug("  %s [Timer::terminate()][%s->%s]" % (self.name, self.status, new_status))
        # clear the time if finishing with SUCCESS or in the case of an interruption from INVALID
        if new_status == common.Status.SUCCESS or new_status == common.Status.INVALID:
            self.finish_time = None


@meta.oneshot
class OneshotTimer(Timer):
    pass
