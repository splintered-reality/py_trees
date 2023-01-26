#!/usr/bin/env python
#
# License: BSD
#   https://raw.githubusercontent.com/splintered-reality/py_trees/devel/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""The core behaviour template for all py_tree behaviours."""

##############################################################################
# Imports
##############################################################################

from __future__ import annotations

import abc
import re
import typing
import uuid

from . import blackboard, common, logging

##############################################################################
# Behaviour BluePrint
##############################################################################


class Behaviour(abc.ABC):
    """A parent class for all user definable tree behaviours.

    Args:
        name: the behaviour name, defaults to auto-generating from the class name

    Raises:
        TypeError: if the provided name is not a string

    Attributes:
        ~py_trees.behaviours.Behaviour.id (:class:`uuid.UUID`): automagically generated unique identifier
            for the behaviour
        ~py_trees.behaviours.Behaviour.name (:obj:`str`): the behaviour name
        ~py_trees.behaviours.Behaviour.blackboards (typing.List[py_trees.blackboard.Client]): collection of attached
            blackboard clients
        ~py_trees.behaviours.Behaviour.status (:class:`~py_trees.common.Status`): the behaviour status
            (:data:`~py_trees.common.Status.INVALID`,
            :data:`~py_trees.common.Status.RUNNING`,
            :data:`~py_trees.common.Status.FAILURE`,
            :data:`~py_trees.common.Status.SUCCESS`)
        ~py_trees.behaviours.Behaviour.parent (:class:`~py_trees.behaviour.Behaviour`): a
            :class:`~py_trees.composites.Composite` instance if nested in a tree, otherwise None
        ~py_trees.behaviours.Behaviour.children ([:class:`~py_trees.behaviour.Behaviour`]): empty for regular
            behaviours, populated for composites
        ~py_trees.behaviours.Behaviour.logger (:class:`logging.Logger`): a simple logging mechanism
        ~py_trees.behaviours.Behaviour.feedback_message(:obj:`str`): improve debugging with a simple message
        ~py_trees.behaviours.Behaviour.blackbox_level (:class:`~py_trees.common.BlackBoxLevel`): a helper variable
            for dot graphs and runtime gui's to collapse/explode entire subtrees dependent upon the blackbox level.

    .. seealso::
       * :ref:`Skeleton Behaviour Template <skeleton-behaviour-include>`
       * :ref:`The Lifecycle Demo <py-trees-demo-behaviour-lifecycle-program>`
       * :ref:`The Action Behaviour Demo <py-trees-demo-action-behaviour-program>`
    """

    def __init__(self, name: str):
        if not isinstance(name, str):
            raise TypeError(
                "a behaviour name should be a string, but you passed in {}".format(
                    type(name)
                )
            )
        self.id = (
            uuid.uuid4()
        )  # used to uniquely identify this node (helps with removing children from a tree)
        self.name: str = name
        self.blackboards: typing.List[blackboard.Client] = []
        self.qualified_name = "{}/{}".format(
            self.__class__.__qualname__, self.name
        )  # convenience
        self.status = common.Status.INVALID
        self.iterator = self.tick()
        self.parent: typing.Optional[
            Behaviour
        ] = None  # will get set if a behaviour is added to a composite
        self.children: typing.List[Behaviour] = []  # only set by composite behaviours
        self.logger = logging.Logger(name)
        self.feedback_message = ""  # useful for debugging, or human readable updates, but not necessary to implement
        self.blackbox_level = common.BlackBoxLevel.NOT_A_BLACKBOX

    ############################################
    # User Customisable Callbacks
    ############################################

    def setup(self, **kwargs: typing.Any) -> None:  # noqa: B027
        """
        Set up and verify infrastructure (middleware connections, etc) is available.

        Users should override this method for any configuration and/or validation
        that is necessary prior to ticking the tree. Such construction is best
        done here rather than in __init__ since there is no guarantee at __init__
        that the infrastructure is ready or even available (e.g. you may be just
        rendering dot graphs of the trees, no robot around).

        Examples:
         * establishing a middleware connection to a sensor or driver
         * ensuring a sensor or driver is in a 'ready' state

        This method will typically be called before a tree's first tick as this gives
        the application time to check and verify that everything is in a ready state before
        executing. This is especially important given that a tree does not always tick
        every behaviour and if not checked up-front, it may be some time before
        discovering a behaviour was in a broken state.

        .. tip::
           When to use :meth:`~py_trees.behaviour.Behaviour.__init__`,
           :meth:`~py_trees.behaviour.Behaviour.setup` and when to use
           :meth:`~py_trees.behaviour.Behaviour.initialise`?

           Use :meth:`~py_trees.behaviour.Behaviour.__init__` for configuration of
           non-runtime dependencies (e.g. no middleware).

           Use :meth:`~py_trees.behaviour.Behaviour.setup` for one-offs or to get
           early signal that everything (e.g. middleware) is ready to go.

           Use :meth:`~py_trees.behaviour.Behaviour.initialise` for just-in-time
           configurations and/or checks.

           There are times when it makes sense to do all three. For example,
           pythonic variable configuration in :meth:`~py_trees.behaviour.Behaviour.__init__`,
           middleware service client creation / server existence checks in
           :meth:`~py_trees.behaviour.Behaviour.setup` and a just-in-time check
           to ensure the server is still available in :meth:`~py_trees.behaviour.Behaviour.initialise`.

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
            **kwargs: distribute arguments to this
               behaviour and in turn, all of it's children

        Raises:
            Exception: if this behaviour has a fault in construction or configuration

        .. seealso:: :meth:`py_trees.behaviour.Behaviour.shutdown`
        """
        pass

    def initialise(self) -> None:  # noqa: B027
        """
        Execute user specified instructions prior to commencement of a new round of activity.

        Users should override this method to perform any necessary initialising/clearing/resetting
        of variables prior to a new round of activity for the behaviour.

        This method is automatically called via the :meth:`py_trees.behaviour.Behaviour.tick` method
        whenever the behaviour is not :data:`~py_trees.common.Status.RUNNING`.

        ... note:: This method can be called more than once in the lifetime of a tree!
        """
        pass

    def terminate(self, new_status: common.Status) -> None:  # noqa: B027
        """
        Execute user specified instructions when the behaviour is stopped.

        Users should override this method to clean up.
        It will be triggered when a behaviour either
        finishes execution (switching from :data:`~py_trees.common.Status.RUNNING`
        to :data:`~py_trees.common.Status.FAILURE` || :data:`~py_trees.common.Status.SUCCESS`)
        or it got interrupted by a higher priority branch (switching to
        :data:`~py_trees.common.Status.INVALID`). Remember that
        the :meth:`~py_trees.behaviour.Behaviour.initialise` method
        will handle resetting of variables before re-entry, so this method is about
        disabling resources until this behaviour's next tick. This could be a indeterminably
        long time. e.g.

        * cancel an external action that got started
        * shut down any temporary communication handles

        Args:
            new_status (:class:`~py_trees.common.Status`): the behaviour is transitioning to this new status

        .. warning:: Do not set `self.status = new_status` here, that is automatically handled
           by the :meth:`~py_trees.behaviour.Behaviour.stop` method.
           Use the argument purely for introspection purposes (e.g.
           comparing the current state in `self.status` with the state it will transition to in
           `new_status`.

        .. seealso:: :meth:`py_trees.behaviour.Behaviour.stop`
        """
        pass

    @abc.abstractmethod
    def update(self) -> common.Status:
        """
        Execute user specified instructions when the behaviour is ticked.

        Users should override this method to perform any logic required to
        arrive at a decision on the behaviour's new status. It is the primary worker function called
        by the :meth:`~py_trees.behaviour.Behaviour.tick` mechanism.

        Returns:
            the behaviour's new status :class:`~py_trees.common.Status`

        .. tip:: This method should be almost instantaneous and non-blocking

        .. seealso:: :meth:`py_trees.behaviour.Behaviour.tick`
        """
        return common.Status.INVALID

    def shutdown(self) -> None:  # noqa: B027
        """
        Destroy setup infrastructure (the antithesis of setup).

        Users should override this method for any custom destruction of infrastructure
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
        self, name: typing.Optional[str] = None, namespace: typing.Optional[str] = None
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
        new_blackboard = blackboard.Client(name=name, namespace=namespace)
        self.blackboards.append(new_blackboard)
        return new_blackboard

    ############################################
    # Public - lifecycle API
    ############################################

    def setup_with_descendants(self) -> None:
        """Call setup on this child, it's children (it's children's children, )."""
        for child in self.children:
            for node in child.iterate():
                node.setup()
        self.setup()

    def tick_once(self) -> None:
        """Tick the object without iterating step-by-step over the children (i.e. without generators)."""
        # no logger necessary here...it directly relays to tick
        for _unused in self.tick():
            pass

    def tick(self) -> typing.Iterator[Behaviour]:
        """
        Tick the behaviour.

        This function is a generator that can be used by an iterator on
        an entire behaviour tree. It handles the logic for deciding when to
        call the user's :meth:`~py_trees.behaviour.Behaviour.initialise`
        and :meth:`~py_trees.behaviour.Behaviour.terminate` methods as well as making the
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
            a reference to itself

        .. warning::
           Users should not override this method to provide custom tick behaviour. The
           :meth:`~py_trees.behaviour.Behaviour.update` method has been provided for that purpose.
        """
        self.logger.debug("%s.tick()" % (self.__class__.__name__))
        if self.status != common.Status.RUNNING:
            self.initialise()
        # don't set self.status yet, terminate() may need to check what the current state is first
        new_status = self.update()
        if new_status not in list(common.Status):
            self.logger.error(
                "A behaviour returned an invalid status, setting to INVALID [%s][%s]"
                % (new_status, self.name)
            )
            new_status = common.Status.INVALID
        if new_status != common.Status.RUNNING:
            self.stop(new_status)
        self.status = new_status
        yield self

    def iterate(self, direct_descendants: bool = False) -> typing.Iterator[Behaviour]:
        """
        Iterate over this child and it's children.

        This utilises python generators for looping. To traverse the entire tree:

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

    # TODO: better type refinement of 'viso=itor'
    def visit(self, visitor: typing.Any) -> None:
        """
        Introspect on this behaviour with a visitor.

        This is functionality that enables external introspection into the behaviour. It gets used
        by the tree manager classes to collect information as ticking traverses a tree.

        Args:
            visitor: the visiting class, must have a run(:class:`~py_trees.behaviour.Behaviour`) method.
        """
        visitor.run(self)

    def stop(self, new_status: common.Status) -> None:
        """
        Stop the behaviour with the specified status.

        Args:
            new_status: the behaviour is transitioning to this new status

        This is called to bring the current round of activity for the behaviour to completion, typically
        resulting in a final status of :data:`~py_trees.common.Status.SUCCESS`,
        :data:`~py_trees.common.Status.FAILURE` or :data:`~py_trees.common.Status.INVALID`.

        .. warning::
           Users should not override this method to provide custom termination behaviour. The
           :meth:`~py_trees.behaviour.Behaviour.terminate` method has been provided for that purpose.
        """
        self.logger.debug(
            "%s.stop(%s)"
            % (
                self.__class__.__name__,
                "%s->%s" % (self.status, new_status)
                if self.status != new_status
                else "%s" % new_status,
            )
        )
        self.terminate(new_status)
        self.status = new_status
        self.iterator = self.tick()

    ############################################
    # Public - introspection API
    ############################################
    def has_parent_with_name(self, name: str) -> bool:
        """
        Search this behaviour's ancestors for one with the specified name.

        Args:
            name: name of the parent to match, can be a regular expression

        Returns:
            whether a parent was found or not
        """
        pattern = re.compile(name)
        b = self
        while b.parent is not None:
            if pattern.match(b.parent.name) is not None:
                return True
            b = b.parent
        return False

    def has_parent_with_instance_type(
        self, instance_type: "typing.Type[Behaviour]"
    ) -> bool:
        """
        Search this behaviour's ancestors for one of the specified type.

        Args:
            instance type of the parent to match

        Returns:
            whether a parent was found or not
        """
        b = self
        while b.parent is not None:
            if isinstance(b.parent, instance_type):
                return True
            b = b.parent
        return False

    def tip(self) -> typing.Optional[Behaviour]:
        """
        Get the *tip* of this behaviour's subtree (if it has one).

        This corresponds to the the deepest node that was running before the
        subtree traversal reversed direction and headed back to this node.

        Returns:
            The deepest node (behaviour) that was running before subtree traversal
            reversed direction, or None if this behaviour's status is
            :data:`~py_trees.common.Status.INVALID`.
        """
        return self if self.status != common.Status.INVALID else None


##############################################################################
# Mypy Convenience Types
##############################################################################


BehaviourSubClass = typing.TypeVar("BehaviourSubClass", bound=Behaviour)
# BehaviourUpdateMethod = typing.Callable[[BehaviourSubClass], common.Status]
