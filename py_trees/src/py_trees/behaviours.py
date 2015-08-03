#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/yujinrobot/gopher_crazy_hospital/license/LICENSE
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

import logging
import uuid

from .common import Status

##############################################################################
# Behaviour BluePrint
##############################################################################


class Behaviour(object):
    """ A node in a behavior tree that uses coroutines in its tick function """

    def __init__(self, name="", *args, **kwargs):
        self.id = uuid.uuid4()  # used to uniquely identify this node (helps with removing children from a tree)
        self.name = name
        self.status = Status.INVALID
        self.iterator = self.tick()
        self.parent = None  # will get set if a behaviour is added to a composite
        self.children = []  # only set by composite behaviours
        self.logger = logging.getLogger("py_trees.Behaviour")

    ############################################
    # User Defined Functions (virtual)
    ############################################

    def initialise(self):
        self.logger.debug("  %s [initialise()]" % self.name)

    def terminate(self, new_status):
        """
        User defined terminate (cleanup) function. For comparison tests, it is
        often useful to check if the current status is Status.RUNNING and
        the new status is different to trigger appropriate cleanup actions.

        :param Status new_status: compare with current status for decision logic.

        """
        self.logger.debug("  %s [Behaviour::terminate()]" % self.name)
        pass

    def update(self):
        """
        User customisable update function called on by the :py:meth:`tick() <py_trees.behaviours.Behaviour.tick>` method.

        :return: the behaviour's current :py:class:`Status <py_trees.common.Status>`
        """
        self.logger.debug("  %s [Behaviour::update()]" % self.name)
        return Status.INVALID

    ############################################
    # Workers
    ############################################

    def visit(self, visitor):
        """
        This is functionality permitting
        visitors to run over the running part of
        a behaviour tree.
        :param visitor: the visiting class, must have a run(Behaviour) method.
        """
        visitor.run(self)

    def tick(self):
        """
        The update mechanism for the behaviour. Customisation goes
        into the update() function.

        :return Status: resulting state after the tick is completed
        """
        self.logger.debug("  %s [Behaviour::tick()]" % self.name)
        if self.status != Status.RUNNING:
            self.initialise()
        # don't set self.status yet, terminate() may need to check what the current state is first
        new_status = self.update()
        if new_status != Status.RUNNING:
            self.abort(new_status)
        self.status = new_status
        yield self

    def iterate(self):
        """
        Generator that provides iteration over this behaviour and all its children.
        """
        for child in self.children:
            for node in child.iterate():
                yield node
        yield self

    def abort(self, new_status=Status.INVALID):
        """
        :param Status new_status: set the status to this once done.

        This calls the user defined terminate() method and also resets the
        generator. The specified status can be used to compare with the
        current status field for decision making within the terminate function
        itself.

        Do not think of this as an abort only for shutdown and invalid state setting
        (though this is the default argument). Abort is also the proper call when
        the behaviour returns success at which point the behaviour must abort
        any processing and do any required cleanup so it doesn't sit around
        consuming resources awaiting the next time it must be called (initialisation
        should not be done here - it could be a long time before the behaviour gets
        called again, and you shouldn't consume those resources in the meantime).
        """
        self.logger.debug("  %s [Behaviour::abort()]" % self.name)
        self.terminate(new_status)
        self.status = new_status
        self.iterator = self.tick()

    ############################################
    # Status
    ############################################

    def get_status(self):
        return self.status

    def set_status(self, s):
        self.status = s

    def is_running(self):
        return self.status == Status.RUNNING

    def is_terminated(self):
        return (self.status == Status.SUCCESS) or (self.status == Status.FAILURE)

##############################################################################
# Behaviour Metaprogramming
##############################################################################


def create_behaviour_from_function(func):
    """
    Create a behaviour from the specified function, swapping it in place of
    the default update function.

    :param func: function to be used in place of the default :py:meth:`Behaviour.update() <py_trees.behaviours.Behaviour.update>` function.
    :type func: any function with just one argument for 'self', must return Status
    """
    class_name = func.__name__.capitalize()
    #globals()[class_name] = type(class_name, (Behaviour,), dict(update=func))
    return type(class_name, (Behaviour,), dict(update=func))


# Inverter Decorator
def invert(func):
    """Inverts the function, used for the inverter decorator."""
    def wrapped(*args, **kwargs):
        status = func(*args, **kwargs)
        return Status.FAILURE if (status == Status.SUCCESS) else Status.SUCCESS
    return wrapped


def behaviour_update_inverter(cls):
    """
    Inverts the result of a class's update function.

    .. code-block:: python

       @behaviour_update_inverter
       class Failure(Success)
           pass

    or

    .. code-block:: python

       failure = behaviour_update_inverter(Success("Failure"))
    """
    update = getattr(cls, "update")
    setattr(cls, "update", invert(update))
    return cls


##############################################################################
# Function Behaviours
##############################################################################


def success(self):
    self.logger.debug("  %s [Success::update()]" % self.name)
    return Status.SUCCESS


def failure(self):
    self.logger.debug("  %s [Success::update()]" % self.name)
    return Status.FAILURE

Success = create_behaviour_from_function(success)
Failure = create_behaviour_from_function(failure)

# Another way of doing this via the inverter decorator
# @behaviour_update_inverter
# class Failure(Success):
#    pass

##############################################################################
# Other Behaviours
##############################################################################


class Periodic(Behaviour):
    """
    Return running for N ticks, success for N ticks, failure for N ticks...
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
                self.response = Status.RUNNING
            elif self.response == Status.RUNNING:
                self.response = Status.SUCCESS
            else:
                self.response = Status.FAILURE
            self.count = 0
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
        self.logger.debug("  %s [SuccessEveryN::update()][%s]" % (self.name, self.count))
        if self.count % self.every_n == 0:
            return Status.SUCCESS
            self.count = 0
        else:
            return Status.FAILURE


class Count(Behaviour):
    def __init__(self, name="Count", fail_until=3, running_until=5, success_until=6, *args, **kwargs):
        super(Count, self).__init__(name, *args, **kwargs)
        self.count = 0
        self.fail_until = fail_until
        self.running_until = running_until
        self.success_until = success_until
        self.number_count_resets = 0
        self.number_updated = 0
        self.logger = logging.getLogger("py_trees.Count")

    def terminate(self, new_status):
        self.logger.debug("  %s [terminate()]" % self.name)
        # reset only if udpate got us into an invalid state
        if new_status == Status.INVALID:
            self.count = 0
            self.number_count_resets += 1

    def update(self):
        self.number_updated += 1
        self.count += 1
        if self.count <= self.fail_until:
            self.logger.debug("  %s [update()][%s -> FAILURE]" % (self.name, self.count))
            return Status.FAILURE
        elif self.count <= self.running_until:
            self.logger.debug("  %s [update()][%s -> RUNNING]" % (self.name, self.count))
            return Status.RUNNING
        elif self.count <= self.success_until:
            self.logger.debug("  %s [update()][%s -> SUCCESS]" % (self.name, self.count))
            return Status.SUCCESS
        else:
            self.logger.debug("  %s [update()][%s -> FAILURE]" % (self.name, self.count))
            return Status.FAILURE

    def __repr__(self):
        s = "%s\n" % self.name
        s += "  Status : %s\n" % self.status
        s += "  Count  : %s\n" % self.count
        s += "  Resets : %s\n" % self.number_count_resets
        s += "  Updates: %s\n" % self.number_updated
        return s
