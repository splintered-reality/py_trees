#!/usr/bin/env python
#
# License: BSD
#   https://raw.githubusercontent.com/stonier/py_trees_suite/devel/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
.. module:: behaviour
   :platform: Unix
   :synopsis: The core behaviour template.

----

"""

##############################################################################
# Imports
##############################################################################

import re
import unique_id

from . import logging
from . import common

from .common import Status

##############################################################################
# Behaviour BluePrint
##############################################################################


class Behaviour(object):
    """
    A node in a behavior tree that uses coroutines in its tick function.
    When implementing (subclassing) your own behaviour, there are four methods you should
    consider implementing. All are optional.

    * :py:meth:`__init__` : minimal one-time initialisation relevant for rendering this behaviour in a tree offline
    * :py:meth:`setup` : delayed one-time initialisation for things like hardware sensors or connection to drivers that would interfere with rendering this behaviour in a tree offline
    * :py:meth:`initialise` : init/clear/reset variables ready for a new run of the behaviour
    * :py:meth:`update` : where the :term:`tick` happens (i.e. work), computes the new behaviour status
    * :py:meth:`terminate` : cleanup required after the behaviour has finished running (:py:data:`~py_trees.common.Status.SUCCESS`/:py:data:`~py_trees.common.Status.FAILURE`) or been interrupted (:py:data:`~py_trees.common.Status.INVALID`)

    :ivar str name: the behaviour name
    :ivar Status status: the behaviour status (:py:data:`~py_trees.common.Status.INVALID`, :py:data:`~py_trees.common.Status.RUNNING`, :py:data:`~py_trees.common.Status.FAILURE`, :py:data:`~py_trees.common.Status.SUCCESS`)
    :ivar Behaviour parent: None of it is the root otherwise usually a :py:class:`~py_trees.composites.Composite`
    :ivar [] children: a :py:class:`Behaviour` list, empty for standalones, but popuated by composites.
    :ivar str feedback_message: a simple message usually set in the :py:meth:`update`.
    :ivar BlackBoxLevel blackbox_level: helper variable for dot graphs and runtime gui's to collapse/explode entire subtrees dependent upon the blackbox level.
    """
    def __init__(self, name="", *args, **kwargs):
        """
        :param str name: the behaviour name (doesn't have to be unique and can include capitals and spaces)
        """
        assert isinstance(name, basestring), "a behaviour name should be a string, but you passed in %s" % type(name)
        self.id = unique_id.fromRandom()  # used to uniquely identify this node (helps with removing children from a tree)
        self.name = name
        self.status = Status.INVALID
        self.iterator = self.tick()
        self.parent = None  # will get set if a behaviour is added to a composite
        self.children = []  # only set by composite behaviours
        self.logger = logging.Logger(name)
        self.feedback_message = ""  # useful for debugging, or human readable updates, but not necessary to implement
        self.blackbox_level = common.BlackBoxLevel.NOT_A_BLACKBOX

    ############################################
    # User Customisable Functions (virtual)
    ############################################

    def setup(self, timeout):
        """
        :param double timeout: time to wait (0.0 is blocking forever)
        :returns: whether it timed out trying to setup
        :rtype: boolean

        **User Customisable Callback**

        Subclasses may implement this method to do any delayed construction that
        is necessary for runtime. This is best done here rather than in the constructor
        so that trees can be instantiated on the fly without any severe runtime requirements
        (e.g. a ros master) to produce visualisations such as dot graphs.

        A good example of its use include any of the ros wait_for methods which
        will block while looking for a ros master/topics/services.
        """
        # user function, no need to report on any activity here - user should do that in his class
        return True

    def initialise(self):
        """
        **User Customisable Callback**

        Subclasses may implement this method to perform any necessary initialising/clearing/resetting
        of variables when when preparing to enter a behaviour. By entry, this means every time
        the behaviour switches from a non-running state (is :py:data:`~py_trees.common.Status.INVALID`,
        or previously had :py:data:`~py_trees.common.Status.FAILURE`||:py:data:`~py_trees.common.Status.SUCCESS`),
        so expect to run more than once!
        """
        pass  # user function, no need to report on any activity here - user should do that in his class

    def terminate(self, new_status):
        """
        :param :py:class:`~py_trees.common.Status` new_status: compare with current status for decision logic.

        **User Customisable Callback**

        Subclasses may implement this method to be triggered when a behaviour either finishes
        execution (switching from :py:data:`~py_trees.common.Status.RUNNING` to
        :py:data:`~py_trees.common.Status.FAILURE` || :py:data:`~py_trees.common.Status.SUCCESS`)
        or it got interrupted by a higher priority branch (switching to
        :py:data:`~py_trees.common.Status.INVALID`). Remember that the :py:meth:`initialise` method
        will handle any resetting of variables before re-entry, so this method is more about cancelling
        and disabling anything that you do not wish to continue running while waiting for the next
        chance to run the behaviour. e.g.

        * cancel an external action that got started
        * shut down any tempoarary communication handles
        """
        pass  # user function, no need to report on any activity here - user should do that in his class

    def update(self):
        """
        **User Customisable Callback**

        User customisable update function called on by the :py:meth:`tick() <py_trees.behaviours.Behaviour.tick>` method.

        :return: the behaviour's current :py:class:`Status <py_trees.common.Status>`
        """
        return Status.INVALID

    ############################################
    # User Methods
    ############################################

    def tick_once(self):
        """
        A direct means of calling tick on this object without using the generator
        mechanism.
        """
        # no logger necessary here...it directly relays to tick
        for unused in self.tick():
            pass

    ############################################
    # Workers
    ############################################
    def has_parent_with_name(self, name):
        """
        Moves up through this behaviour's parents looking for
        a behaviour with the same name as that specified.

        :param str name: name of the parent to match, can be a regular expression
        :returns: true or false depending on whether such a parent was found
        """
        pattern = re.compile(name)
        b = self
        while b.parent is not None:
            if pattern.match(b.parent.name) is not None:
                return True
            b = b.parent
        return False

    def has_parent_with_instance_type(self, instance_type):
        """
        Moves up through this behaviour's parents looking for
        a behaviour with the same instance type as that specified.

        :param str instance_type: instance type of the parent to match
        :returns: true or false depending on whether such a parent was found
        """
        b = self
        while b.parent is not None:
            if isinstance(b.parent, instance_type):
                return True
            b = b.parent
        return False

    def tip(self):
        """
        Get the "tip" of this behaviour's subtree (if it has one).
        This corresponds to the the deepest node that was running before the
        subtree traversal reversed direction and headed back to this node.

        In the case of a standalone behaviour with no children, it
        returns itself if it is not invalid, otherwise None.
        """
        return self if self.status != Status.INVALID else None

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
        This function is a generator that can be used by an iterator on
        an entire behaviour tree. It handles the logic for initialisation and return
        state, but the actual work should go into the implementation of the update() function.
        Once it's worked out the logic, it will then yield itself (generator
        mechanism) so that it can be used as part of an iterator for the
        entire tree.

        .. warning::

           This is a generator function, you must use this with *yield*. If you need a direct call,
           prefer :py:meth:`tick_once` instead.

        Calling ``my_behaviour.tick()`` will not actually do anything.
        It has to be used as part of an iterator, e.g.

        .. code-block:: python

           for node in my_behaviour.tick():
               print("Do something")

        Prefer instead :py:function::`tick_once` method for making a direct call.

        :return py_trees.Behaviour: a reference to itself
        """
        self.logger.debug("%s.tick()" % (self.__class__.__name__))
        if self.status != Status.RUNNING:
            self.initialise()
        # don't set self.status yet, terminate() may need to check what the current state is first
        new_status = self.update()
        if new_status not in list(Status):
            self.logger.error("A behaviour returned an invalid status, setting to INVALID [%s][%s]" % (new_status, self.name))
            new_status = Status.INVALID
        if new_status != Status.RUNNING:
            self.stop(new_status)
        self.status = new_status
        yield self

    def iterate(self, direct_descendants=False):
        """
        Generator that provides iteration over this behaviour and all its children.
        """
        for child in self.children:
            if not direct_descendants:
                for node in child.iterate():
                    yield node
            else:
                yield child
        yield self

    def stop(self, new_status=Status.INVALID):
        """
        :param Status new_status: set the status to this once done.

        This calls the user defined terminate() method and also resets the
        generator. The specified status can be used to compare with the
        current status field for decision making within the terminate function
        itself.

        .. warning:: Do not override this method, use :py:meth:`terminate` instead.
        """
        self.logger.debug("%s.stop()[%s]" % (self.__class__.__name__, "%s->%s" % (self.status, new_status) if self.status != new_status else "%s" % new_status))
        self.terminate(new_status)
        self.status = new_status
        self.iterator = self.tick()
