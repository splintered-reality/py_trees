#!/usr/bin/env python
#
# License: BSD
#   https://raw.githubusercontent.com/stonier/py_trees/devel/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
Decorators
"""

##############################################################################
# Imports
##############################################################################

from .behaviour import Behaviour
from .common import Status

import time


##############################################################################
# Decorators
##############################################################################

class Decorator(Behaviour):
    """
    this enforces one child only
    """
    def __init__(self, decorated, name="Decorator"):
        # later addition of decorated? like composites..
        self.decorated = decorated
        super(Decorator, self).__init__(name=name)

    def setup(self, timeout):
        self.logger.debug("%s.setup()" % (self.__class__.__name__))
        result = self.decorated.setup(timeout)
        if result is None:
            # replace with py_trees exception!
            self.logger.error("%s.setup()['%s'.setup() returned None (must be True||False)]" % (self.__class__.__name__, self.decorated.name))

        return result

    def tick(self):
        self.logger.debug("%s.tick()" % self.__class__.__name__)
        # Required behaviour for *all* behaviours and composites is
        # for tick() to check if it isn't running and initialise
        if self.status != Status.RUNNING:
            # run subclass (user) handles
            self.initialise()

        # process all including the decorated
        for node in self.decorated.tick():
            yield node

        # run any work designated by a customised instance of this class
        new_status = self.update()
        if new_status not in list(Status):
            self.logger.error("A behaviour returned an invalid status, setting to INVALID [%s][%s]" % (new_status, self.name))
            new_status = Status.INVALID
        self.status = new_status
        yield self

    def update(self):
        return self.decorated.status

    def stop(self, new_status=Status.INVALID):
        self.decorated.stop(new_status)
        super(Decorator, self).stop(new_status)


class Timeout(Decorator):
    def __init__(self, decorated, duration):
        super(Timeout, self).__init__(decorated)
        self.duration = duration
        self.finish_time = None

    def initialise(self):
        if self.finish_time is None:
            self.finish_time = time.time() + self.duration

    def update(self):
        # make sure this class initialises time related functions
        # when the underlying original will initialise itself
        if self.decorated.status != Status.RUNNING:
            self.initialise()
        current_time = time.time()
        if current_time > self.finish_time:
            self.feedback_message = "timed out"
            # invalidate the decorated (i.e. cancel it)
            self.decorated.stop(Status.INVALID)
            return Status.FAILURE
        self.feedback_message = self.decorated.feedback_message + " [time left: %s]" % (self.finish_time - current_time)
        return self.decorated.status

    def terminate(self, new_status):
        if new_status != Status.RUNNING:
            self.finish_time = None


class Inverter(Decorator):
    def update(self):
        if self.decorated.status == Status.SUCCESS:
            return Status.FAILURE
            self.feedback_message = "success -> failure"
        elif self.decorated.status == Status.FAILURE:
            self.feedback_message = "failure -> success"
            return Status.SUCCESS
        self.feedback_message = self.decorated.feedback_message
        return self.decorated.status


class OneShot(Decorator):
    def __init__(self, decorated):
        super(OneShot, self).__init__(decorated)
        self.final_status = None

    def update(self):
        self.logger.debug("OneShot.wrapped_update()")
        if self.final_status:
            return self.final_status
        if self.decorated.status in (Status.FAILURE, Status.SUCCESS):
            self.final_status = self.decorated.status
        return self.decorated.status

    def tick(self):
        if self.final_status:
            self.status = self.final_status
            self.logger.debug("OneShot.wrapped_tick()[rebounding]")
            yield self
        else:
            self.logger.debug("OneShot.wrapped_tick()")
            for behaviour in super(OneShot, self).tick():
                yield behaviour

    def terminate(self, new_status):
        self.logger.debug("OneShot.wrapped_terminate()[{}]".format(new_status))
        # handle only the interrupt/reset case
        if new_status == Status.INVALID:
            if self.final_status:
                self.status = new_status
            else:
                self.decorated.stop(new_status)
                self.status = self.decorated.status


class RunningIsFailure(Decorator):
    def update(self):
        if self.decorated.status == Status.RUNNING:
            self.feedback_message = "running is failure" + (" [%s]" % self.decorated.feedback_message if self.decorated.feedback_message else "")
            return Status.FAILURE
        self.feedback_message = self.decorated.feedback_message
        return self.decorated.status


class RunningIsSuccess(Decorator):
    def update(self):
        if self.decorated.status == Status.RUNNING:
            self.feedback_message = "running is success" + (" [%s]" % self.decorated.feedback_message if self.decorated.feedback_message else "")
            return Status.SUCCESS
        self.feedback_message = self.decorated.feedback_message
        return self.decorated.status


class FailureIsSuccess(Decorator):
    def update(self):
        if self.decorated.status == Status.FAILURE:
            self.feedback_message = "failure is success" + (" [%s]" % self.decorated.feedback_message if self.decorated.feedback_message else "")
            return Status.SUCCESS
        self.feedback_message = self.decorated.feedback_message
        return self.decorated.status


class FailureIsRunning(Decorator):
    def update(self):
        if self.decorated.status == Status.FAILURE:
            self.feedback_message = "failure is running" + (" [%s]" % self.decorated.feedback_message if self.decorated.feedback_message else "")
            return Status.RUNNING
        self.feedback_message = self.decorated.feedback_message
        return self.decorated.status


class SuccessIsFailure(Decorator):
    def update(self):
        if self.decorated.status == Status.SUCCESS:
            self.feedback_message = "success is failure" + (" [%s]" % self.decorated.feedback_message if self.decorated.feedback_message else "")
            return Status.FAILURE
        self.feedback_message = self.decorated.feedback_message
        return self.decorated.status


class SuccessIsRunning(Decorator):
    def update(self):
        if self.decorated.status == Status.SUCCESS:
            self.feedback_message = "success is running [%s]" % self.decorated.feedback_message
            return Status.RUNNING
        self.feedback_message = self.decorated.feedback_message
        return self.decorated.status


class Condition(Decorator):
    def __init__(self, decorated, status):
        super(Condition, self).__init__(decorated)
        self.succeed_status = status

    def update(self):
        self.logger.debug("%s.update()" % self.__class__.__name__)
        self.feedback_message = "'{0}' has status {1}, waiting for {2}".format(self.decorated.name, self.decorated.status, self.succeed_status)
        if self.decorated.status == self.succeed_status:
            if self.decorated.status == Status.RUNNING:
                self.decorated.stop()
            return Status.SUCCESS
        return Status.RUNNING

