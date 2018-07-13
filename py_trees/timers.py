#
# License: Yujin
#
##############################################################################
# Description
##############################################################################

"""
Time related behaviours.
"""

##############################################################################
# Imports
##############################################################################

import numbers
import time

from . import behaviour
from . import common
from . import meta

##############################################################################
# Behaviours
##############################################################################


class Timer(behaviour.Behaviour):
    """
    Simple timer class that is :py:data:`~py_trees.common.Status.RUNNING` until the timer
    runs out, at which point it is :data:`~py_trees.common.Status.SUCCESS`. This can be
    used in a wide variety of situations - pause, duration, timeout depending on how
    it is wired into the tree (e.g. pause in a sequence, duration/timeout in
    a parallel).

    The timer gets reset either upon entry (:meth:`~py_trees.behaviour.Behaviour.initialise`)
    if it hasn't already been set and gets cleared when it either runs out, or the behaviour is
    interrupted by a higher priority or parent cancelling it.

    Args:
        name (:obj:`str`): name of the behaviour
        duration (:obj:`int`): length of time to run (in seconds)

    .. note::
        This succeeds the first time the behaviour is ticked **after** the expected
        finishing time.

    .. tip::
        Use the :func:`~py_trees.meta.running_is_failure` decorator if you need
        :data:`~py_trees.common.Status.FAILURE` until the timer finishes.

    """
    def __init__(self, name="Timer", duration=5.0):
        super(Timer, self).__init__(name)
        assert isinstance(duration, numbers.Real), "Timer: duration should be int or float, but you passed in %s" % type(duration)
        self.duration = duration
        self.finish_time = None
        self.feedback_message = "duration set to '{0}'s".format(self.duration)

    def initialise(self):
        """
        Store the expected finishing time.
        """
        self.logger.debug("%s.initialise()" % self.__class__.__name__)
        if self.finish_time is None:
            self.finish_time = time.time() + self.duration
        self.feedback_message = "configured to fire in '{0}' seconds".format(self.duration)

    def update(self):
        """
        Check current time against the expected finishing time. If it is in excess, flip to
        :data:`~py_trees.common.Status.SUCCESS`.
        """
        self.logger.debug("%s.update()" % self.__class__.__name__)
        current_time = time.time()
        if current_time > self.finish_time:
            self.feedback_message = "timer ran out [{0}]".format(self.duration)
            return common.Status.SUCCESS
        else:
            # do not show the time, it causes the tree to be 'changed' every tick
            # and we don't want to spam visualisations with almost meaningless updates
            self.feedback_message = "still running"  # (%s)" % (self.finish_time - current_time)
            return common.Status.RUNNING

    def terminate(self, new_status):
        """
        Clear the expected finishing time.
        """
        self.logger.debug("%s.terminate(%s)" % (self.__class__.__name__, "%s->%s" % (self.status, new_status) if self.status != new_status else "%s" % new_status))
        # clear the time if finishing with SUCCESS or in the case of an interruption from INVALID
        if new_status == common.Status.SUCCESS or new_status == common.Status.INVALID:
            self.finish_time = None


@meta.oneshot
class OneshotTimer(Timer):
    pass
