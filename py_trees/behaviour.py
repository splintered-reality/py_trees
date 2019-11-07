#!/usr/bin/env python
#
# License: BSD
#   https://raw.githubusercontent.com/splintered-reality/py_trees/devel/LICENSE
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

from . import blackboard
from . import common
from . import logging

##############################################################################
# Behaviour BluePrint
##############################################################################


class Behaviour(object):
    """
    Defines the basic properties and methods required of a node in a
    behaviour tree. When implementing your own behaviour,
    subclass this class.

    Args:
        name: the behaviour name, defaults to auto-generating from the class name

    Raises:
        TypeError: if the provided name is not a string

    Attributes:
        ~py_trees.behaviours.Behaviour.id (:class:`uuid.UUID`): automagically generated unique identifier for the behaviour
        ~py_trees.behaviours.Behaviour.name (:obj:`str`): the behaviour name
        ~py_trees.behaviours.Behaviour.blackboards (typing.List[py_trees.blackboard.Client]): collection of attached blackboard clients
        ~py_trees.behaviours.Behaviour.status (:class:`~py_trees.common.Status`): the behaviour status (:data:`~py_trees.common.Status.INVALID`, :data:`~py_trees.common.Status.RUNNING`, :data:`~py_trees.common.Status.FAILURE`, :data:`~py_trees.common.Status.SUCCESS`)
        ~py_trees.behaviours.Behaviour.parent (:class:`~py_trees.behaviour.Behaviour`): a :class:`~py_trees.composites.Composite` instance if nested in a tree, otherwise None
        ~py_trees.behaviours.Behaviour.children ([:class:`~py_trees.behaviour.Behaviour`]): empty for regular behaviours, populated for composites
        ~py_trees.behaviours.Behaviour.logger (:class:`logging.Logger`): a simple logging mechanism
        ~py_trees.behaviours.Behaviour.feedback_message(:obj:`str`): improve debugging with a simple message
        ~py_trees.behaviours.Behaviour.blackbox_level (:class:`~py_trees.common.BlackBoxLevel`): a helper variable for dot graphs and runtime gui's to collapse/explode entire subtrees dependent upon the blackbox level.

    .. seealso::
       * :ref:`Skeleton Behaviour Template <skeleton-behaviour-include>`
       * :ref:`The Lifecycle Demo <py-trees-demo-behaviour-lifecycle-program>`
       * :ref:`The Action Behaviour Demo <py-trees-demo-action-behaviour-program>`

    """
    def __init__(self,
                 name: str=common.Name.AUTO_GENERATED):
        if not name or name == common.Name.AUTO_GENERATED:
            name = self.__class__.__name__
        if not isinstance(name, str):
            raise TypeError("a behaviour name should be a string, but you passed in {}".format(type(name)))
        self.id = uuid.uuid4()  # used to uniquely identify this node (helps with removing children from a tree)
        self.name = name
        self.blackboards = []
        self.qualified_name = "{}/{}".format(self.__class__.__qualname__, self.name)  # convenience
        self.status = common.Status.INVALID
        self.iterator = self.tick()
        self.parent = None  # will get set if a behaviour is added to a composite
        self.children = []  # only set by composite behaviours
        self.logger = logging.Logger(name)
        self.feedback_message = ""  # useful for debugging, or human readable updates, but not necessary to implement
        self.blackbox_level = common.BlackBoxLevel.NOT_A_BLACKBOX

    ############################################
    # User Customisable Callbacks
    ############################################

    def setup(self, **kwargs):
        """
        .. note:: User Customisable Callback

        Subclasses may override this method for any one-off delayed construction &
        validation that is necessary prior to ticking the tree. Such construction is best
        done here rather than in __init__ so that trees can be instantiated on the fly for
        easy rendering to dot graphs without imposing runtime requirements (e.g. establishing
        a middleware connection to a sensor or a driver to a serial port).

        Equally as important, executing methods which validate the configuration of
        behaviours will increase confidence that your tree will successfully tick
        without logical software errors before actually ticking. This is useful both
        before a tree's first tick and immediately after any modifications to a tree
        has been made between ticks.

        .. tip::

           Faults are notified to the user of the behaviour via exceptions.
           Choice of exception to use is left to the user.

        .. warning::

           The kwargs argument is for distributing objects at runtime to behaviours
           before ticking. For example, a simulator instance with which behaviours can
           interact with the simulator's python api, a ros2 node for setting up
           communications. Use sparingly, as this is not proof against keyword conflicts
           amongst disparate libraries of behaviours.

        Args:
            **kwargs (:obj:`dict`): distribute arguments to this
               behaviour and in turn, all of it's children

        Raises:
            Exception: if this behaviour has a fault in construction or configuration

        .. seealso:: :meth:`py_trees.behaviour.Behaviour.shutdown`
        """
        pass

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
        return common.Status.INVALID

    def shutdown(self):
        """
        .. note:: User Customisable Callback

        Subclasses may override this method for any custom destruction of infrastructure
        usually brought into being in :meth:`~py_trees.behaviour.Behaviour.setup`.

        Raises:
            Exception: of whatever flavour the child raises when errors occur on destruction

        .. seealso:: :meth:`py_trees.behaviour.Behaviour.setup`
        """
        pass

    ############################################
    # Private Methods - use inside a behaviour
    ############################################

    def attach_blackboard_client(
            self,
            name: str=None,
            namespace: str=None
    ) -> blackboard.Client:
        """
        Create and attach a blackboard to this behaviour.

        Args:
            name: human-readable (not necessarily unique) name for the client
            namespace: sandbox the client to variables behind this namespace

        Returns:
            a handle to the attached blackboard client
        """
        if name is None:
            count = len(self.blackboards)
            name = self.name if (count == 0) else self.name + "-{}".format(count)
        new_blackboard = blackboard.Client(
            name=name,
            namespace=namespace
        )
        self.blackboards.append(new_blackboard)
        return new_blackboard

    ############################################
    # Public - lifecycle API
    ############################################

    def setup_with_descendants(self):
        """
        Iterates over this child, it's children (it's children's children, ...)
        calling the user defined :meth:`~py_trees.behaviour.Behaviour.setup`
        on each in turn.
        """
        for child in self.children:
            for node in child.iterate():
                node.setup()
        self.setup()

    def tick_once(self):
        """
        A direct means of calling tick on this object without
        using the generator mechanism.
        """
        # no logger necessary here...it directly relays to tick
        for unused in self.tick():
            pass

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

        .. warning:: Override this method only in exceptional circumstances, prefer overriding :meth:`~py_trees.behaviour.Behaviour.update` instead.

        """
        self.logger.debug("%s.tick()" % (self.__class__.__name__))
        if self.status != common.Status.RUNNING:
            self.initialise()
        # don't set self.status yet, terminate() may need to check what the current state is first
        new_status = self.update()
        if new_status not in list(common.Status):
            self.logger.error("A behaviour returned an invalid status, setting to INVALID [%s][%s]" % (new_status, self.name))
            new_status = common.Status.INVALID
        if new_status != common.Status.RUNNING:
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

    def visit(self, visitor):
        """
        This is functionality that enables external introspection into the behaviour. It gets used
        by the tree manager classes to collect information as ticking traverses a tree.

        Args:
            visitor (:obj:`object`): the visiting class, must have a run(:class:`~py_trees.behaviour.Behaviour`) method.
        """
        visitor.run(self)

    def stop(self, new_status=common.Status.INVALID):
        """
        Args:
            new_status (:class:`~py_trees.common.Status`): the behaviour is transitioning to this new status

        This calls the user defined :meth:`~py_trees.behaviour.Behaviour.terminate` method and also resets the
        generator. It will finally set the new status once the user's :meth:`~py_trees.behaviour.Behaviour.terminate`
        function has been called.

        .. warning:: Override this method only in exceptional circumstances, prefer overriding :meth:`~py_trees.behaviour.Behaviour.terminate` instead.
        """
        self.logger.debug("%s.stop(%s)" % (self.__class__.__name__, "%s->%s" % (self.status, new_status) if self.status != new_status else "%s" % new_status))
        self.terminate(new_status)
        self.status = new_status
        self.iterator = self.tick()

    ############################################
    # Public - introspection API
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
        return self if self.status != common.Status.INVALID else None

    def verbose_info_string(self):
        """
        Override to provide a one line informative string about the behaviour. This
        gets used in, e.g. dot graph rendering of the tree.

        .. tip::
           Use this sparingly. A good use case is for when the behaviour type
           and class name isn't sufficient to inform the user about it's
           mechanisms for controlling the flow of a tree tick (e.g. parallels
           with policies).
        """
        return ""
