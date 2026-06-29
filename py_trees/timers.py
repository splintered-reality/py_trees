#!/usr/bin/env python
#
# License: BSD
#   https://raw.githubusercontent.com/splintered-reality/py_trees/devel/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""Time related behaviours."""

##############################################################################
# Imports
##############################################################################

import time

from . import behaviour, common

##############################################################################
# Behaviours
##############################################################################


class Timer(behaviour.Behaviour):
    """
    A simple, blocking timer behaviour running off python time.time().

    This behaviour is :py:data:`~py_trees.common.Status.RUNNING` until the timer
    runs out, at which point it is :data:`~py_trees.common.Status.SUCCESS`. This can be
    used in a wide variety of situations - pause, duration, timeout depending on how
    it is wired into the tree (e.g. pause in a sequence, duration/timeout in
    a parallel).

    The timer gets reset either upon entry (:meth:`~py_trees.behaviour.Behaviour.initialise`)
    if it hasn't already been set and gets cleared when it either runs out, or the behaviour is
    interrupted by a higher priority or parent cancelling it.

    Args:
        name: name of the behaviour
        duration: length of time to run (in seconds)

    Raises:
        TypeError: if the provided duration is not a real number

    .. note::
        This succeeds the first time the behaviour is ticked **after** the expected
        finishing time.

    .. tip::
        Use the :func:`~py_trees.decorators.RunningIsFailure` decorator if you need
        :data:`~py_trees.common.Status.FAILURE` until the timer finishes.
    """

    def __init__(self, name: str = "Timer", duration: float = 5.0):
        super().__init__(name)
        if not isinstance(duration, (int, float)):
            raise TypeError(f"Timer: duration should be int or float, but you passed in {type(duration)}")
        self.duration: float = duration
        self.finish_time: float = 0.0
        self.feedback_message: str = f"duration set to '{self.duration}'s"

    def initialise(self) -> None:
        """Store the expected finishing time."""
        self.logger.debug(f"{self.__class__.__name__}.initialise()")
        self.finish_time = time.time() + self.duration
        self.feedback_message = f"configured to fire in '{self.duration}' seconds"

    def update(self) -> common.Status:
        """
        Check the timer and update the behaviour result accordingly.

        Returns:
            :data:`~py_trees.common.Status.RUNNING` until timer expires, then
            :data:`~py_trees.common.Status.SUCCESS`.

        """
        self.logger.debug(f"{self.__class__.__name__}.update()")
        current_time = time.time()
        if current_time > self.finish_time:
            self.feedback_message = f"timer ran out [{self.duration}]"
            return common.Status.SUCCESS
        else:
            # do not show the time, it causes the tree to be 'changed' every tick
            # and we don't want to spam visualisations with almost meaningless updates
            self.feedback_message = "still running"  # (%s)" % (self.finish_time - current_time)
            return common.Status.RUNNING
