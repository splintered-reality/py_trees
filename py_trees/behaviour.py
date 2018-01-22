#!/usr/bin/env python
#
# License: BSD
#   https://raw.githubusercontent.com/stonier/py_trees/devel/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
The core behaviour template. All behaviours, standalone and composite, inherit
from this class.
"""

##############################################################################
# Imports
##############################################################################

import re
import uuid

from . import logging
from . import common

from .common import Status

##############################################################################
# Behaviour BluePrint
##############################################################################


class Behaviour(object):
    """
    Defines the basic properties and methods required of a node in a
    behaviour tree.

    Uses all the whizbang tricks from coroutines and generators to do this
    as optimally as you may in python. When implementing your own behaviour,
    subclass this class.

    Args:
        name (:obj:`str`): the behaviour name
        *args: variable length argument list.
        **kwargs: arbitrary keyword arguments.

    Attributes:
        name (:obj:`str`): the behaviour name
        status (:class:`~py_trees.common.Status`): the behaviour status (:data:`~py_trees.common.Status.INVALID`, :data:`~py_trees.common.Status.RUNNING`, :data:`~py_trees.common.Status.FAILURE`, :data:`~py_trees.common.Status.SUCCESS`)
        parent (:class:`~py_trees.behaviour.Behaviour`): a :class:`~py_trees.composites.Composite` instance if nested in a tree, otherwise None
        children ([:class:`~py_trees.behaviour.Behaviour`]): empty for regular behaviours, populated for composites
        feedback_message(:obj:`str`): a simple message used to notify of significant happenings
        blackbox_level (:class:`~py_trees.common.BlackBoxLevel`): a helper variable for dot graphs and runtime gui's to collapse/explode entire subtrees dependent upon the blackbox level.

    .. seealso::
       * :ref:`Skeleton Behaviour Template <skeleton-behaviour-include>`
       * :ref:`The Lifecycle Demo <py-trees-demo-behaviour-lifecycle-program>`
       * :ref:`The Action Behaviour Demo <py-trees-demo-action-behaviour-program>`

    """
    def __init__(self, name="", *args, **kwargs):
        try:
            assert isinstance(name, basestring), "a behaviour name should be a string, but you passed in %s" % type(name)  # python2 compatibility
        except NameError:
            assert isinstance(name, str), "a behaviour name should be a string, but you passed in %s" % type(name)
        self.id = uuid.uuid4()  # used to uniquely identify this node (helps with removing children from a tree)
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
        Subclasses may override this method to do any one-time delayed construction that
        is necessary for runtime. This is best done here rather than in the constructor
        so that trees can be instantiated on the fly without any severe runtime requirements
        (e.g. a hardware sensor) on any pc to produce visualisations such as dot graphs.

        .. note:: User Customisable Callback

        Args:
            timeout (:obj:`float`): time to wait (0.0 is blocking forever)

        Returns:
            :obj:`bool`: whether it timed out trying to setup
        """
        return True

    def initialise(self):
        """
        .. note:: User Customisable Callback

        Subclasses may override this method to perform any necessary initialising/clearing/resetting
        of variables when when preparing to enter this behaviour if it was not previously
        :data:`~py_trees.common.Status.RUNNING`. i.e. Expect this to trigger more than once!
        """
        pass

    def terminate(self, new_status):
        """
        .. note:: User Customisable Callback

        Subclasses may override this method to clean up. It will be triggered when a behaviour either
        finishes execution (switching from :data:`~py_trees.common.Status.RUNNING`
        to :data:`~py_trees.common.Status.FAILURE` || :data:`~py_trees.common.Status.SUCCESS`)
        or it got interrupted by a higher priority branch (switching to
        :data:`~py_trees.common.Status.INVALID`). Remember that the :meth:`~py_trees.behaviour.Behaviour.initialise` method
        will handle resetting of variables before re-entry, so this method is about
        disabling resources until this behaviour's next tick. This could be a indeterminably
        long time. e.g.

        * cancel an external action that got started
        * shut down any tempoarary communication handles

        Args:
            new_status (:class:`~py_trees.common.Status`): the behaviour is transitioning to this new status

        .. warning:: Do not set `self.status = new_status` here, that is automatically handled
           by the :meth:`~py_trees.behaviour.Behaviour.stop` method. Use the argument purely for introspection purposes (e.g.
           comparing the current state in `self.status` with the state it will transition to in
           `new_status`.
        """
        pass

    def update(self):
        """
        .. note:: User Customisable Callback

        Returns:
            :class:`~py_trees.common.Status`: the behaviour's new status :class:`~py_trees.common.Status`

        Subclasses may override this method to perform any logic required to
        arrive at a decision on the behaviour's new status. It is the primary worker function called on
        by the :meth:`~py_trees.behaviour.Behaviour.tick` mechanism.

        .. tip:: This method should be almost instantaneous and non-blocking

        """
        return Status.INVALID

    ############################################
    # User Methods
    ############################################

    def tick_once(self):
        """
        A direct means of calling tick on this object without
        using the generator mechanism.
        """
        # no logger necessary here...it directly relays to tick
        for unused in self.tick():
            pass

    ############################################
    # Workers
    ############################################
    def has_parent_with_name(self, name):
        """
        Searches through this behaviour's parents, and their parents, looking for
        a behaviour with the same name as that specified.

        Args:
            name (:obj:`str`): name of the parent to match, can be a regular expression

        Returns:
            bool: whether a parent was found or not
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

        Args:
            instance_type (:obj:`str`): instance type of the parent to match

        Returns:
            bool: whether a parent was found or not
        """
        b = self
        while b.parent is not None:
            if isinstance(b.parent, instance_type):
                return True
            b = b.parent
        return False

    def tip(self):
        """
        Get the *tip* of this behaviour's subtree (if it has one) after it's last
        tick. This corresponds to the the deepest node that was running before the
        subtree traversal reversed direction and headed back to this node.

        Returns:
            :class:`~py_trees.behaviour.Behaviour` or :obj:`None`: child behaviour, itself or :obj:`None` if its status is :data:`~py_trees.common.Status.INVALID`
        """
        return self if self.status != Status.INVALID else None

    def visit(self, visitor):
        """
        This is functionality that enables external introspection into the behaviour. It gets used
        by the tree manager classes to collect information as ticking traverses a tree.

        Args:
            visitor (:obj:`object`): the visiting class, must have a run(:class:`~py_trees.behaviour.Behaviour`) method.
        """
        visitor.run(self)

    def tick(self):
        """
        This function is a generator that can be used by an iterator on
        an entire behaviour tree. It handles the logic for deciding when to
        call the user's :meth:`~py_trees.behaviour.Behaviour.initialise` and :meth:`~py_trees.behaviour.Behaviour.terminate` methods as well as making the
        actual call to the user's :meth:`~py_trees.behaviour.Behaviour.update` method that determines the
        behaviour's new status once the tick has finished. Once done, it will
        then yield itself (generator mechanism) so that it can be used as part of
        an iterator for the entire tree.

        .. code-block:: python

           for node in my_behaviour.tick():
               print("Do something")

        .. note::

           This is a generator function, you must use this with *yield*. If you need a direct call,
           prefer :meth:`~py_trees.behaviour.Behaviour.tick_once` instead.

        Yields:
            :class:`~py_trees.behaviour.Behaviour`: a reference to itself
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
        To traverse the entire tree:

        .. code-block:: python

           for node in my_behaviour.iterate():
               print("Name: {0}".format(node.name))

        Args:
            direct_descendants (:obj:`bool`): only yield children one step away from this behaviour.

        Yields:
            :class:`~py_trees.behaviour.Behaviour`: one of it's children
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
        Args:
            new_status (:class:`~py_trees.common.Status`): the behaviour is transitioning to this new status

        This calls the user defined :meth:`~py_trees.behaviour.Behaviour.terminate` method and also resets the
        generator. It will finally set the new status once the user's :meth:`~py_trees.behaviour.Behaviour.terminate`
        function has been called.

        .. warning:: Do not use this method, override :meth:`~py_trees.behaviour.Behaviour.terminate` instead.
        """
        self.logger.debug("%s.stop(%s)" % (self.__class__.__name__, "%s->%s" % (self.status, new_status) if self.status != new_status else "%s" % new_status))
        self.terminate(new_status)
        self.status = new_status
        self.iterator = self.tick()
