#!/usr/bin/env python
#
# License: BSD
#   https://raw.githubusercontent.com/splintered-reality/py_trees/devel/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
Composites (multi-child) types for behaviour trees.

Composites are responsible for directing the path traced through
the tree on a given tick (execution). They are the **factories**
(Sequences and Parallels) and **decision makers** (Selectors) of a behaviour
tree.

.. graphviz:: dot/composites.dot
   :align: center
   :caption: PyTree Composites

Composite behaviours typically manage children and apply some logic to the way
they execute and return a result, but generally don't do anything themselves.
Perform the checks or actions you need to do in the non-composite behaviours.

Most any desired functionality can be authored with a combination of these
three composites. In fact, it is precisely this feature that makes behaviour
trees attractive - it breaks down complex decision making logic to just three
primitive elements. It is possible and often desirable to extend this set with
custom composites of your own, but think carefully before you do - in almost
every case, a combination of the existing composites will serve and as a
result, you will merely compound the complexity inherent in your tree logic.
This this makes it confoundingly difficult to design, introspect and debug. As
an example, design sessions often revolve around a sketched graph on a
whiteboard. When these graphs are composed of just five elements (Selectors,
Sequences, Parallels, Decorators and Behaviours), it is very easy to understand
the logic at a glance. Double the number of fundamental elements and you may as
well be back at the terminal parsing code.

.. tip:: You should never need to subclass or create new composites.

The basic operational modes of the three composites in this library are as follows:

* :class:`~py_trees.composites.Selector`: execute a child based on cascading priorities
* :class:`~py_trees.composites.Sequence`: execute children sequentially
* :class:`~py_trees.composites.Parallel`: execute children concurrently

This library does provide some flexibility in *how* each composite is implemented without
breaking the fundamental nature of each (as described above). Selectors and Sequences can
be configured with or without memory (resumes or resets if children are RUNNING) and
the results of a parallel can be configured to wait upon all children completing, succeed
on one, all or a subset thereof.

.. tip:: Follow the links in each composite's documentation to the relevant demo programs.

"""

##############################################################################
# Imports
##############################################################################

import abc
import itertools
import typing
import uuid

from . import behaviour, common

##############################################################################
# Composites
##############################################################################


class Composite(behaviour.Behaviour, abc.ABC):
    """
    The parent class to all composite behaviours.

    Args:
        name (:obj:`str`): the composite behaviour name
        children ([:class:`~py_trees.behaviour.Behaviour`]): list of children to add
    """

    def __init__(
        self,
        name: str,
        children: typing.Optional[typing.List[behaviour.Behaviour]] = None,
    ):
        super(Composite, self).__init__(name)
        if children is not None:
            for child in children:
                self.add_child(child)
        else:
            self.children = []
        self.current_child: typing.Optional[behaviour.Behaviour] = None

    ############################################
    # Virtual
    ############################################
    @abc.abstractmethod
    def tick(self) -> typing.Iterator[behaviour.Behaviour]:
        """
        Tick the composite.

        All composite subclasses require a re-implementation
        of the tick method to provide the logic for managing multiple
        children (:meth:`~py_trees.behaviour.Behaviour.tick` merely
        provides default logic for when there are no children).
        """
        pass

    ############################################
    # Unused
    ############################################

    def update(self) -> common.Status:
        """
        Unused update method.

        Composites should direct the flow, whilst
        behaviours do the real work.

        Such flows are a consequence of how the composite
        interacts with it's children. The success of
        behaviour trees depends on this logic being simple,
        well defined and limited to a few well established
        patterns - this is what ensures that visualising
        a tree enables a user to quickly grasp the
        decision making captured therein.

        For the standard patterns, this logic is limited
        to the ordering of execution and logical inferences
        on the resulting status of the composite's children.

        This is a good guideline to adhere to (i.e. don't reach
        inside children to inference on custom variables, nor
        reach out to the system your tree is attached to).

        Implementation wise, this renders the
        :meth:`~py_trees.behaviour.Behaviour.update` method redundant
        as all customisation to create a simple, well defined composite
        happens in the :meth:`~py_trees.behaviour.Behaviour.tick` method.

        Bottom line, composites do not make use of this method.
        Implementing it for subclasses of the core composites
        will not do anything.
        """
        return common.Status.INVALID

    ############################################
    # Overrides
    ############################################

    def stop(self, new_status: common.Status = common.Status.INVALID) -> None:
        """
        Provide common stop-level functionality for all composites.

         * Retain the current child on :data:`~py_trees.common.Status.SUCCESS` or
           :data:`~py_trees.common.Status.FAILURE` (for introspection), lose it on
           :data:`~py_trees.common.Status.INVALID`
         * Kill dangling (:data:`~py_trees.common.Status.RUNNING`) children

        The latter situation can arise for some composites, but more importantly,
        will always occur when high higher priority behaviour interrupts this one.

        Args:
            new_status: behaviour will transition to this new status
        """
        # Priority interrupt handling
        if new_status == common.Status.INVALID:
            self.current_child = None
            for child in self.children:
                if (
                    child.status != common.Status.INVALID
                ):  # redundant if INVALID->INVALID
                    child.stop(new_status)

        # Regular Behaviour.stop() handling
        #   could call directly, but replicating here to avoid repeating the logger
        self.terminate(new_status)
        self.status = new_status
        self.iterator = self.tick()

    def tip(self) -> typing.Optional[behaviour.Behaviour]:
        """
        Recursive function to extract the last running node of the tree.

        Returns:
            the tip function of the current child of this composite or None
        """
        if self.current_child is not None:
            return self.current_child.tip()
        else:
            return super().tip()

    ############################################
    # Children
    ############################################

    def add_child(self, child: behaviour.Behaviour) -> uuid.UUID:
        """
        Add a child.

        Args:
            child: child to add

        Raises:
            TypeError: if the child is not an instance of :class:`~py_trees.behaviour.Behaviour`
            RuntimeError: if the child already has a parent

        Returns:
            unique id of the child
        """
        if not isinstance(child, behaviour.Behaviour):
            raise TypeError(
                "children must be behaviours, but you passed in {}".format(type(child))
            )
        self.children.append(child)
        if child.parent is not None:
            raise RuntimeError(
                "behaviour '{}' already has parent '{}'".format(
                    child.name, child.parent.name
                )
            )
        child.parent = self
        return child.id

    def add_children(
        self, children: typing.List[behaviour.Behaviour]
    ) -> behaviour.Behaviour:
        """
        Append a list of children to the current list.

        Args:
            children ([:class:`~py_trees.behaviour.Behaviour`]): list of children to add
        """
        for child in children:
            self.add_child(child)
        return self

    def remove_child(self, child: behaviour.Behaviour) -> int:
        """
        Remove the child behaviour from this composite.

        Args:
            child: child to delete

        Returns:
            index of the child that was removed

        .. todo:: Error handling for when child is not in this list
        """
        if self.current_child is not None and (self.current_child.id == child.id):
            self.current_child = None
        if child.status == common.Status.RUNNING:
            child.stop(common.Status.INVALID)
        child_index = self.children.index(child)
        self.children.remove(child)
        child.parent = None
        return child_index

    def remove_all_children(self) -> None:
        """Remove all children. Makes sure to stop each child if necessary."""
        self.current_child = None
        for child in self.children:
            if child.status == common.Status.RUNNING:
                child.stop(common.Status.INVALID)
            child.parent = None
        # makes sure to delete it for this class and all references to it
        #   http://stackoverflow.com/questions/850795/clearing-python-lists
        del self.children[:]

    def replace_child(
        self, child: behaviour.Behaviour, replacement: behaviour.Behaviour
    ) -> None:
        """
        Replace the child behaviour with another.

        Args:
            child: child to delete
            replacement: child to insert
        """
        self.logger.debug(
            "%s.replace_child()[%s->%s]"
            % (self.__class__.__name__, child.name, replacement.name)
        )
        child_index = self.children.index(child)
        self.remove_child(child)
        self.insert_child(replacement, child_index)
        child.parent = None

    def remove_child_by_id(self, child_id: uuid.UUID) -> None:
        """
        Remove the child with the specified id.

        Args:
            child_id: unique id of the child

        Raises:
            IndexError: if the child was not found
        """
        child = next((c for c in self.children if c.id == child_id), None)
        if child is not None:
            self.remove_child(child)
        else:
            raise IndexError(
                "child was not found with the specified id [%s]" % child_id
            )

    def prepend_child(self, child: behaviour.Behaviour) -> uuid.UUID:
        """
        Prepend the child before all other children.

        Args:
            child: child to insert

        Returns:
            uuid.UUID: unique id of the child
        """
        self.children.insert(0, child)
        child.parent = self
        return child.id

    def insert_child(self, child: behaviour.Behaviour, index: int) -> uuid.UUID:
        """
        Insert child at the specified index.

        This simply directly calls the python list's :obj:`insert` method using the child and index arguments.

        Args:
            child (:class:`~py_trees.behaviour.Behaviour`): child to insert
            index (:obj:`int`): index to insert it at

        Returns:
            uuid.UUID: unique id of the child
        """
        self.children.insert(index, child)
        child.parent = self
        return child.id


##############################################################################
# Selector
##############################################################################


class Selector(Composite):
    """
    Selectors are the decision makers.

    .. graphviz:: dot/selector.dot

    A selector executes each of its child behaviours in turn until one of them
    succeeds (at which point it itself returns :data:`~py_trees.common.Status.RUNNING`
    or :data:`~py_trees.common.Status.SUCCESS`,
    or it runs out of children at which point it itself returns :data:`~py_trees.common.Status.FAILURE`.
    We usually refer to selecting children as a means of *choosing between priorities*.
    Each child and its subtree represent a decreasingly lower priority path.

    .. note::

       Switching from a low -> high priority branch causes a `stop(INVALID)` signal to be sent to the previously
       executing low priority branch. This signal will percolate down that child's own subtree. Behaviours
       should make sure that they catch this and *destruct* appropriately.

    .. note::

       If configured with `memory`, higher priority checks will be skipped when a child returned with
       running on the previous tick. i.e. once a priority is locked in, it will run to completion and can
       only be interrupted if the selector is interrupted by higher priorities elsewhere in the tree.

    .. seealso:: The :ref:`py-trees-demo-selector-program` program demos higher priority switching under a selector.

    Args:
        memory (:obj:`bool`): if :data:`~py_trees.common.Status.RUNNING` on the previous tick,
            resume with the :data:`~py_trees.common.Status.RUNNING` child
        name (:obj:`str`): the composite behaviour name
        children ([:class:`~py_trees.behaviour.Behaviour`]): list of children to add
    """

    def __init__(
        self,
        name: str,
        memory: bool,
        children: typing.Optional[typing.List[behaviour.Behaviour]] = None,
    ):
        super(Selector, self).__init__(name, children)
        self.memory = memory

    def tick(self) -> typing.Iterator[behaviour.Behaviour]:
        """
        Customise the tick behaviour for a selector.

        This implements priority-interrupt style handling amongst the selector's children.
        The selector's status is always a reflection of it's children's status.

        Yields:
            :class:`~py_trees.behaviour.Behaviour`: a reference to itself or one of its children
        """
        self.logger.debug("%s.tick()" % self.__class__.__name__)
        # initialise
        if self.status != common.Status.RUNNING:
            # selector specific initialisation - leave initialise() free for users to
            # re-implement without having to make calls to super()
            self.logger.debug(
                "%s.tick() [!RUNNING->reset current_child]" % self.__class__.__name__
            )
            self.current_child = self.children[0] if self.children else None

            # reset the children - don't need to worry since they will be handled
            # a) prior to a remembered starting point, or
            # b) invalidated by a higher level priority

            # user specific initialisation
            self.initialise()

        # nothing to do
        if not self.children:
            self.current_child = None
            self.stop(common.Status.FAILURE)
            yield self
            return

        # starting point
        if self.memory:
            assert self.current_child is not None  # should never be true, help mypy out
            index = self.children.index(self.current_child)
            # clear out preceding status' - not actually necessary but helps
            # visualise the case of memory vs no memory
            for child in itertools.islice(self.children, None, index):
                child.stop(common.Status.INVALID)
        else:
            index = 0

        # actual work
        previous = self.current_child
        for child in itertools.islice(self.children, index, None):
            for node in child.tick():
                yield node
                if node is child:
                    if (
                        node.status == common.Status.RUNNING
                        or node.status == common.Status.SUCCESS
                    ):
                        self.current_child = child
                        self.status = node.status
                        if previous is None or previous != self.current_child:
                            # we interrupted, invalidate everything at a lower priority
                            passed = False
                            for child in self.children:
                                if passed:
                                    if child.status != common.Status.INVALID:
                                        child.stop(common.Status.INVALID)
                                passed = True if child == self.current_child else passed
                        yield self
                        return
        # all children failed, set failure ourselves and current child to the last bugger who failed us
        self.status = common.Status.FAILURE
        try:
            self.current_child = self.children[-1]
        except IndexError:
            self.current_child = None
        yield self

    def stop(self, new_status: common.Status = common.Status.INVALID) -> None:
        """
        Ensure that children are appropriately stopped and update status.

        Args:
            new_status : the composite is transitioning to this new status
        """
        self.logger.debug(
            f"{self.__class__.__name__}.stop()[{self.status}->{new_status}]"
        )
        Composite.stop(self, new_status)


##############################################################################
# Sequence
##############################################################################


class Sequence(Composite):
    """
    Sequences are the factory lines of behaviour trees.

    .. graphviz:: dot/sequence.dot

    A sequence will progressively tick over each of its children so long as
    each child returns :data:`~py_trees.common.Status.SUCCESS`. If any child returns
    :data:`~py_trees.common.Status.FAILURE` or :data:`~py_trees.common.Status.RUNNING`
    the sequence will halt and the parent will adopt
    the result of this child. If it reaches the last child, it returns with
    that result regardless.

    .. note::

       The sequence halts once it engages with a child is RUNNING, remaining behaviours
       are not ticked.

    .. note::

       If configured with `memory` and a child returned with running on the previous tick, it will
       proceed directly to the running behaviour, skipping any and all preceding behaviours. With memory
       is useful for moving through a long running series of tasks. Without memory is useful if you
       want conditional guards in place preceding the work that you always want checked off.

    .. seealso:: The :ref:`py-trees-demo-sequence-program` program demos a simple sequence in action.

    Args:
        name: the composite behaviour name
        memory: if :data:`~py_trees.common.Status.RUNNING` on the previous tick,
            resume with the :data:`~py_trees.common.Status.RUNNING` child
        children: list of children to add
    """

    def __init__(
        self,
        name: str,
        memory: bool,
        children: typing.Optional[typing.List[behaviour.Behaviour]] = None,
    ):
        super(Sequence, self).__init__(name, children)
        self.memory = memory

    def tick(self) -> typing.Iterator[behaviour.Behaviour]:
        """
        Tick over the children.

        Yields:
            :class:`~py_trees.behaviour.Behaviour`: a reference to itself or one of its children
        """
        self.logger.debug("%s.tick()" % self.__class__.__name__)

        # initialise
        index = 0
        if self.status != common.Status.RUNNING:
            self.current_child = self.children[0] if self.children else None
            for child in self.children:
                if child.status != common.Status.INVALID:
                    child.stop(common.Status.INVALID)
            self.initialise()  # user specific initialisation
        elif self.memory and common.Status.RUNNING:
            assert self.current_child is not None  # should never be true, help mypy out
            index = self.children.index(self.current_child)
        elif not self.memory and common.Status.RUNNING:
            self.current_child = self.children[0] if self.children else None
        else:
            # previous conditional checks should cover all variations
            raise RuntimeError("Sequence reached an unknown / invalid state")

        # nothing to do
        if not self.children:
            self.current_child = None
            self.stop(common.Status.SUCCESS)
            yield self
            return

        # actual work
        for child in itertools.islice(self.children, index, None):
            for node in child.tick():
                yield node
                if node is child and node.status != common.Status.SUCCESS:
                    self.status = node.status
                    if not self.memory:
                        # invalidate the remainder of the sequence
                        # i.e. kill dangling runners
                        for child in itertools.islice(self.children, index + 1, None):
                            if child.status != common.Status.INVALID:
                                child.stop(common.Status.INVALID)
                    yield self
                    return
            try:
                # advance if there is 'next' sibling
                self.current_child = self.children[index + 1]
                index += 1
            except IndexError:
                pass

        self.stop(common.Status.SUCCESS)
        yield self

    def stop(self, new_status: common.Status = common.Status.INVALID) -> None:
        """
        Ensure that children are appropriately stopped and update status.

        Args:
            new_status : the composite is transitioning to this new status
        """
        self.logger.debug(
            f"{self.__class__.__name__}.stop()[{self.status}->{new_status}]"
        )
        Composite.stop(self, new_status)


##############################################################################
# Parallel
##############################################################################


class Parallel(Composite):
    """
    Parallels enable a kind of spooky at-a-distance concurrency.

    .. graphviz:: dot/parallel.dot

    A parallel ticks every child every time the parallel is itself ticked.
    The parallelism however, is merely conceptual. The children have actually been
    sequentially ticked, but from both the tree and the parallel's purview, all
    children have been ticked at once.

    The parallelism too, is not true in the sense that it kicks off multiple threads
    or processes to do work. Some behaviours *may* kick off threads or processes
    in the background, or connect to existing threads/processes. The behaviour itself
    however, merely monitors these and is itself encosced in a py_tree which only ever
    ticks in a single-threaded operation.

    * Parallels will return :data:`~py_trees.common.Status.FAILURE` if any
      child returns :py:data:`~py_trees.common.Status.FAILURE`
    * Parallels with policy :class:`~py_trees.common.ParallelPolicy.SuccessOnAll`
      only returns :py:data:`~py_trees.common.Status.SUCCESS` if **all** children
      return :py:data:`~py_trees.common.Status.SUCCESS`
    * Parallels with policy :class:`~py_trees.common.ParallelPolicy.SuccessOnOne`
      return :py:data:`~py_trees.common.Status.SUCCESS` if **at least one** child
      returns :py:data:`~py_trees.common.Status.SUCCESS` and others are
      :py:data:`~py_trees.common.Status.RUNNING`
    * Parallels with policy :class:`~py_trees.common.ParallelPolicy.SuccessOnSelected`
      only returns :py:data:`~py_trees.common.Status.SUCCESS` if a **specified subset**
      of children return :py:data:`~py_trees.common.Status.SUCCESS`

    Policies :class:`~py_trees.common.ParallelPolicy.SuccessOnAll` and
    :class:`~py_trees.common.ParallelPolicy.SuccessOnSelected` may be configured to be
    *synchronised* in which case children that tick with
    :data:`~py_trees.common.Status.SUCCESS` will be skipped on subsequent ticks until
    the policy criteria is met, or one of the children returns
    status :data:`~py_trees.common.Status.FAILURE`.

    Parallels with policy :class:`~py_trees.common.ParallelPolicy.SuccessOnSelected` will
    check in both the :meth:`~py_trees.behaviour.Behaviour.setup` and
    :meth:`~py_trees.behaviour.Behaviour.tick` methods to to verify the
    selected set of children is actually a subset of the children of this parallel.

    .. seealso::
       * :ref:`Context Switching Demo <py-trees-demo-context-switching-program>`
    """

    def __init__(
        self,
        name: str,
        policy: common.ParallelPolicy.Base,
        children: typing.Optional[typing.List[behaviour.Behaviour]] = None,
    ):
        """
        Initialise the behaviour with name, policy and a list of children.

        Args:
            name: the composite behaviour name
            policy: policy for deciding success or otherwise (default: SuccessOnAll)
            children: list of children to add
        """
        super(Parallel, self).__init__(name, children)
        self.policy = policy

    def setup(self, **kwargs: int) -> None:
        """
        Detect before ticking whether the policy configuration is invalid.

        Args:
            **kwargs (:obj:`dict`): distribute arguments to this
               behaviour and in turn, all of it's children

        Raises:
            RuntimeError: if the parallel's policy configuration is invalid
            Exception: be ready to catch if any of the children raise an exception
        """
        self.logger.debug("%s.setup()" % (self.__class__.__name__))
        self.validate_policy_configuration()

    def tick(self) -> typing.Iterator[behaviour.Behaviour]:
        """
        Tick over the children.

        Yields:
            :class:`~py_trees.behaviour.Behaviour`: a reference to itself or one of its children

        Raises:
            RuntimeError: if the policy configuration was invalid
        """
        self.logger.debug("%s.tick()" % self.__class__.__name__)
        self.validate_policy_configuration()

        # reset
        if self.status != common.Status.RUNNING:
            self.logger.debug("%s.tick(): re-initialising" % self.__class__.__name__)
            for child in self.children:
                # reset the children, this ensures old SUCCESS/FAILURE status flags
                # don't break the synchronisation logic below
                if child.status != common.Status.INVALID:
                    child.stop(common.Status.INVALID)
            self.current_child = None
            # subclass (user) handling
            self.initialise()

        # nothing to do
        if not self.children:
            self.current_child = None
            self.stop(common.Status.SUCCESS)
            yield self
            return

        # process them all first
        for child in self.children:
            if self.policy.synchronise and child.status == common.Status.SUCCESS:
                continue
            for node in child.tick():
                yield node

        # determine new status
        new_status = common.Status.RUNNING
        self.current_child = self.children[-1]
        try:
            failed_child = next(
                child
                for child in self.children
                if child.status == common.Status.FAILURE
            )
            self.current_child = failed_child
            new_status = common.Status.FAILURE
        except StopIteration:
            if type(self.policy) is common.ParallelPolicy.SuccessOnAll:
                if all([c.status == common.Status.SUCCESS for c in self.children]):
                    new_status = common.Status.SUCCESS
                    self.current_child = self.children[-1]
            elif type(self.policy) is common.ParallelPolicy.SuccessOnOne:
                successful = [
                    child
                    for child in self.children
                    if child.status == common.Status.SUCCESS
                ]
                if successful:
                    new_status = common.Status.SUCCESS
                    self.current_child = successful[-1]
            elif type(self.policy) is common.ParallelPolicy.SuccessOnSelected:
                if all(
                    [c.status == common.Status.SUCCESS for c in self.policy.children]
                ):
                    new_status = common.Status.SUCCESS
                    self.current_child = self.policy.children[-1]
            else:
                raise RuntimeError(
                    "this parallel has been configured with an unrecognised policy [{}]".format(
                        type(self.policy)
                    )
                )
        # this parallel may have children that are still running
        # so if the parallel itself has reached a final status, then
        # these running children need to be terminated so they don't dangle
        if new_status != common.Status.RUNNING:
            self.stop(new_status)
        self.status = new_status
        yield self

    def stop(self, new_status: common.Status = common.Status.INVALID) -> None:
        """
        Ensure that any running children are stopped.

        Args:
            new_status : the composite is transitioning to this new status
        """
        self.logger.debug(
            f"{self.__class__.__name__}.stop()[{self.status}->{new_status}]"
        )

        # clean up dangling (running) children
        for child in self.children:
            if child.status == common.Status.RUNNING:
                # this unfortunately knocks out it's running status for introspection
                # but logically is the correct thing to do, see #132.
                child.stop(common.Status.INVALID)
        Composite.stop(self, new_status)

    def validate_policy_configuration(self) -> None:
        """
        Validate the currently stored policy.

        Policy configuration can be invalid if:
        * Policy is SuccessOnSelected and no behaviours have been specified
        * Policy is SuccessOnSelected and behaviours that are not children exist

        Raises:
            RuntimeError: if policy configuration was invalid
        """
        if type(self.policy) is common.ParallelPolicy.SuccessOnSelected:
            if not self.policy.children:
                error_message = (
                    "policy SuccessOnSelected requires a non-empty "
                    "selection of children [{}]".format(self.name)
                )
                self.logger.error(error_message)
                raise RuntimeError(error_message)
            missing_children_names = [
                child.name
                for child in self.policy.children
                if child not in self.children
            ]

            if missing_children_names:
                error_message = (
                    "policy SuccessOnSelected has selected behaviours that are "
                    "not children of this parallel {}[{}]"
                    "".format(missing_children_names, self.name)
                )
                self.logger.error(error_message)
                raise RuntimeError(error_message)
