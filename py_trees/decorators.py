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
    pass


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
    pass


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
    pass
