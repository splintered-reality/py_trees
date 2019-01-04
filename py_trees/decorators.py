#!/usr/bin/env python
#
# License: BSD
#   https://raw.githubusercontent.com/stonier/py_trees/devel/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
Decorators are behaviours that manage a single child and provide common
modifications to their underlying child behaviour (e.g. inverting the result).
i.e. they provide a means for behaviours to wear different 'hats' depending
on their context without a behaviour tree.

.. image:: images/many-hats.png
   :width: 40px
   :align: center
"""

##############################################################################
# Imports
##############################################################################

import functools
import time

from . import behaviour
from . import behaviours
from . import common

##############################################################################
# Classes
##############################################################################

class Decorator(behaviour.Behaviour):
    """
    A decorator is responsible for handling the lifecycle of a single
    child beneath
    """
    def __init__(self, name, child):
        """
        Common initialisation steps for a decorator - type checks and
        name construction (if None is given).
        
        Args:
            name (:obj:`str`): the decorator name (can be None)
            children (:class:`~py_trees.behaviour.Behaviour`): the child to be decorated

        Raises:
            TypeError: if the child is not an instance of :class:`~py_trees.behaviour.Behaviour`

        Return:
            :obj:`bool`: suceess or failure of the operation
        """
        # Checks
        if not isinstance(child, behaviour.Behaviour):
            raise TypeError("A decorator's child must be an instance of py_trees.behaviours.Behaviour")
        # Construct an informative name if none is provided 
        if not name or name == common.Name.AUTO_GENERATED:
            name = self.__class__.__name__ + "\n[{}]".format(child.name)
        # Initialise
        super(Decorator, self).__init__(name=name)
        self.children.append(child)
        # Give a convenient alias
        self.decorated = self.children[0]
 
    def setup(self, timeout):
        """
        Relays to the decorated child's :meth:`~py_trees.behaviour.Behaviuor.setup`
        method.

        Args:
             timeout (:obj:`float`): time to wait (0.0 is blocking forever)

        Raises:
            TypeError: if children's setup methods fail to return a boolean

        Return:
            :obj:`bool`: suceess or failure of the operation
        """
        self.logger.debug("%s.setup()" % (self.__class__.__name__))
        result = self.decorated.setup(timeout)
        if type(result) != bool:
            message = "invalid return type from child's setup method (should be bool) [child:'{}'][type:'{}']".format(
                self.decorated.name, type(result))
            raise TypeError(message)
        return result
 
    def tick(self):
        """
        A decorator's tick is exactly the same as a normal proceedings for
        a Behaviour's tick except that it also ticks the decorated child node.

        Yields:
            :class:`~py_trees.behaviour.Behaviour`: a reference to itself or one of its children
        """
        self.logger.debug("%s.tick()" % self.__class__.__name__)
        # initialise just like other behaviours/composites
        if self.status != common.Status.RUNNING:
            self.initialise()
        # interrupt proceedings and process the child node
        # (including any children it may have as well)
        for node in self.decorated.tick():
            yield node
        # resume normal proceedings for a Behaviour's tick
        new_status = self.update()
        if new_status not in list(common.Status):
            self.logger.error("A behaviour returned an invalid status, setting to INVALID [%s][%s]" % (new_status, self.name))
            new_status = common.Status.INVALID
        if new_status != common.Status.RUNNING:
            self.stop(new_status)
        self.status = new_status
        yield self

    def stop(self, new_status):
        """
        As with other composites, it checks if the child is running
        and stops it if that is the case. 

        Args:
            new_status (:class:`~py_trees.common.Status`): the behaviour is transitioning to this new status
        """
        self.logger.debug("%s.terminate()[%s]" % (self.__class__.__name__, new_status))
        if self.decorated.status == common.Status.RUNNING:
            self.decorated.stop(new_status)
 
##############################################################################
# Decorators
##############################################################################

class Timeout(Decorator):
    """
    A decorator that applies a timeout pattern to an existing behaviour.
    If the timeout is reached, the encapsulated behaviour's
    :meth:`~py_trees.behaviour.Behaviour.stop` method is called with
    status :data:`~py_trees.common.Status.FAILURE` otherwise it will
    simply directly tick and return with the same status
    as that of it's encapsulated behaviour.

    Args:
        child (:class:`~py_trees.behaviour.Behaviour`): behaviour to time
        name (:obj:`str`): the decorator name (can be None)
        duration (:obj:`float`): timeout length in seconds

    Returns:
        :class:`~py_trees.behaviour.Behaviour`: the modified behaviour class with timeout
    """
    def __init__(self,
                 child,
                 name=common.Name.AUTO_GENERATED,
                 duration=5.0):
        super(Timeout, self).__init__(name=name, child=child)
        self.duration = duration
        self.finish_time = None

    def initialise(self):
        self.finish_time = time.time() + self.duration
        self.feedback_message = ""
 
    def update(self):
        current_time = time.time()
        if current_time > self.finish_time:
            self.feedback_message = "timed out"
            # invalidate the decorated (i.e. cancel it), could also put this logic in a terminate() method
            self.decorated.stop(common.Status.INVALID)
            return common.Status.FAILURE
        # Don't show the time remaining, that will change the message every tick and make the tree hard to
        # debug since it will record a continuous stream of events
        self.feedback_message = self.decorated.feedback_message + " [timeout: {}]".format(self.finish_time)
        return self.decorated.status
