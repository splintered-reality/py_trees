#!/usr/bin/env python
#
# License: BSD
#   https://raw.githubusercontent.com/splintered-reality/py_trees/devel/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
Composites are the **factories** and **decision makers** of a
behaviour tree. They are responsible for shaping the branches.

.. graphviz:: dot/composites.dot

.. tip:: You should never need to subclass or create new composites.

Most patterns can be achieved with a combination of the above. Adding to this
set exponentially increases the complexity and subsequently
making it more difficult to design, introspect, visualise and debug the trees. Always try
to find the combination you need to achieve your result before contemplating adding
to this set. Actually, scratch that...just don't contemplate it!

Composite behaviours typically manage children and apply some logic to the way
they execute and return a result, but generally don't do anything themselves.
Perform the checks or actions you need to do in the non-composite behaviours.

* :class:`~py_trees.composites.Sequence`: execute children sequentially
* :class:`~py_trees.composites.Selector`: select a path through the tree, interruptible by higher priorities
* :class:`~py_trees.composites.Chooser`: like a selector, but commits to a path once started until it finishes
* :class:`~py_trees.composites.Parallel`: manage children concurrently
"""

##############################################################################
# Imports
##############################################################################

import itertools
import typing

from . import behaviour
from . import common
from .common import Status

##############################################################################
# Composites
##############################################################################


class Composite(behaviour.Behaviour):
    """
    The parent class to all composite behaviours, i.e. those that
    have children.

    Args:
        name (:obj:`str`): the composite behaviour name
        children ([:class:`~py_trees.behaviour.Behaviour`]): list of children to add
    """
    def __init__(self,
                 name: str=common.Name.AUTO_GENERATED,
                 children: typing.List[behaviour.Behaviour]=None
                 ):
        super(Composite, self).__init__(name)
        if children is not None:
            for child in children:
                self.add_child(child)
        else:
            self.children = []

    ############################################
    # Worker Overrides
    ############################################

    def stop(self, new_status=Status.INVALID):
        """
        There is generally two use cases that must be supported here.

        1) Whenever the composite has gone to a recognised state (i.e. :data:`~py_trees.common.Status.FAILURE` or SUCCESS),
        or 2) when a higher level parent calls on it to truly stop (INVALID).

        In only the latter case will children need to be forcibly stopped as well. In the first case, they will
        have stopped themselves appropriately already.

        Args:
            new_status (:class:`~py_trees.common.Status`): behaviour will transition to this new status
        """
        self.logger.debug("%s.stop()[%s]" % (self.__class__.__name__, "%s->%s" % (self.status, new_status) if self.status != new_status else "%s" % new_status))
        if new_status == Status.INVALID:
            for child in self.children:
                child.stop(new_status)
        # This part just replicates the Behaviour.stop function. We replicate it here so that
        # the Behaviour logging doesn't duplicate the composite logging here, just a bit cleaner this way.
        self.terminate(new_status)
        self.status = new_status
        self.iterator = self.tick()

    def tip(self):
        """
        Recursive function to extract the last running node of the tree.

        Returns:
            :class::`~py_trees.behaviour.Behaviour`: the tip function of the current child of this composite or None
        """
        if self.current_child is not None:
            return self.current_child.tip()
        else:
            return super().tip()

    ############################################
    # Children
    ############################################

    def add_child(self, child):
        """
        Adds a child.

        Args:
            child (:class:`~py_trees.behaviour.Behaviour`): child to add

        Raises:
            TypeError: if the provided child is not an instance of :class:`~py_trees.behaviour.Behaviour`

        Returns:
            uuid.UUID: unique id of the child
        """
        if not isinstance(child, behaviour.Behaviour):
            raise TypeError("children must be behaviours, but you passed in {}".format(type(child)))
        self.children.append(child)
        child.parent = self
        return child.id

    def add_children(self, children):
        """
        Append a list of children to the current list.

        Args:
            children ([:class:`~py_trees.behaviour.Behaviour`]): list of children to add
        """
        for child in children:
            self.add_child(child)

    def remove_child(self, child):
        """
        Remove the child behaviour from this composite.

        Args:
            child (:class:`~py_trees.behaviour.Behaviour`): child to delete

        Returns:
            :obj:`int`: index of the child that was removed

        .. todo:: Error handling for when child is not in this list
        """
        if child.status == Status.RUNNING:
            child.stop(Status.INVALID)
        child_index = self.children.index(child)
        self.children.remove(child)
        child.parent = None
        return child_index

    def remove_all_children(self):
        """
        Remove all children. Makes sure to stop each child if necessary.
        """
        for child in self.children:
            if child.status == Status.RUNNING:
                child.stop(Status.INVALID)
            child.parent = None
        # makes sure to delete it for this class and all references to it
        #   http://stackoverflow.com/questions/850795/clearing-python-lists
        del self.children[:]

    def replace_child(self, child, replacement):
        """
        Replace the child behaviour with another.

        Args:
            child (:class:`~py_trees.behaviour.Behaviour`): child to delete
            replacement (:class:`~py_trees.behaviour.Behaviour`): child to insert
        """
        if child.status == Status.RUNNING:
            child.stop(Status.INVALID)
        child_index = self.children.index(child)
        self.logger.debug("%s.replace_child()[%s->%s]" % (self.__class__.__name__, child.name, replacement.name))
        self.children[child_index] = replacement
        child.parent = None
        replacement.parent = self

    def remove_child_by_id(self, child_id):
        """
        Remove the child with the specified id.

        Args:
            child_id (uuid.UUID): unique id of the child

        Raises:
            IndexError: if the child was not found
        """
        child = next((c for c in self.children if c.id == child_id), None)
        if child is not None:
            if child.status == Status.RUNNING:
                child.stop(Status.INVALID)
            self.children.remove(child)
            child.parent = None
        else:
            raise IndexError('child was not found with the specified id [%s]' % child_id)

    def prepend_child(self, child):
        """
        Prepend the child before all other children.

        Args:
            child (:class:`~py_trees.behaviour.Behaviour`): child to insert

        Returns:
            uuid.UUID: unique id of the child
        """
        self.children.insert(0, child)
        child.parent = self
        return child.id

    def insert_child(self, child, index):
        """
        Insert child at the specified index. This simply directly calls
        the python list's :obj:`insert` method using the child and index arguments.

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
    Selectors are the Decision Makers

    .. graphviz:: dot/selector.dot

    A selector executes each of its child behaviours in turn until one of them
    succeeds (at which point it itself returns :data:`~py_trees.common.Status.RUNNING` or :data:`~py_trees.common.Status.SUCCESS`,
    or it runs out of children at which point it itself returns :data:`~py_trees.common.Status.FAILURE`.
    We usually refer to selecting children as a means of *choosing between priorities*.
    Each child and its subtree represent a decreasingly lower priority path.

    .. note::

       Switching from a low -> high priority branch causes a `stop(INVALID)` signal to be sent to the previously
       executing low priority branch. This signal will percolate down that child's own subtree. Behaviours
       should make sure that they catch this and *destruct* appropriately.

    Make sure you do your appropriate cleanup in the :meth:`terminate()` methods! e.g. cancelling a running goal, or restoring a context.

    .. seealso:: The :ref:`py-trees-demo-selector-program` program demos higher priority switching under a selector.

    Args:
        name (:obj:`str`): the composite behaviour name
        children ([:class:`~py_trees.behaviour.Behaviour`]): list of children to add
    """

    def __init__(self, name="Selector", children=None):
        super(Selector, self).__init__(name, children)
        self.current_child = None

    def tick(self):
        """
        Run the tick behaviour for this selector. Note that the status
        of the tick is always determined by its children, not
        by the user customised update function.

        Yields:
            :class:`~py_trees.behaviour.Behaviour`: a reference to itself or one of its children
        """
        self.logger.debug("%s.tick()" % self.__class__.__name__)
        # Required behaviour for *all* behaviours and composites is
        # for tick() to check if it isn't running and initialise
        if self.status != Status.RUNNING:
            # selectors dont do anything specific on initialisation
            #   - the current child is managed by the update, never needs to be 'initialised'
            # run subclass (user) handles
            self.initialise()
        # run any work designated by a customised instance of this class
        self.update()
        previous = self.current_child
        for child in self.children:
            for node in child.tick():
                yield node
                if node is child:
                    if node.status == Status.RUNNING or node.status == Status.SUCCESS:
                        self.current_child = child
                        self.status = node.status
                        if previous is None or previous != self.current_child:
                            # we interrupted, invalidate everything at a lower priority
                            passed = False
                            for child in self.children:
                                if passed:
                                    if child.status != Status.INVALID:
                                        child.stop(Status.INVALID)
                                passed = True if child == self.current_child else passed
                        yield self
                        return
        # all children failed, set failure ourselves and current child to the last bugger who failed us
        self.status = Status.FAILURE
        try:
            self.current_child = self.children[-1]
        except IndexError:
            self.current_child = None
        yield self

    def stop(self, new_status=Status.INVALID):
        """
        Stopping a selector requires setting the current child to none. Note that it
        is important to implement this here instead of terminate, so users are free
        to subclass this easily with their own terminate and not have to remember
        that they need to call this function manually.

        Args:
            new_status (:class:`~py_trees.common.Status`): the composite is transitioning to this new status
        """
        # retain information about the last running child if the new status is
        # SUCCESS or FAILURE
        if new_status == Status.INVALID:
            self.current_child = None
        Composite.stop(self, new_status)

    def __repr__(self):
        """
        Simple string representation of the object.

        Returns:
            :obj:`str`: string representation
        """
        s = "Name       : %s\n" % self.name
        s += "  Status  : %s\n" % self.status
        s += "  Current : %s\n" % (self.current_child.name if self.current_child is not None else "none")
        s += "  Children: %s\n" % [child.name for child in self.children]
        return s


##############################################################################
# Chooser
##############################################################################


class Chooser(Selector):
    """
    Choosers are Selectors with Commitment

    .. graphviz:: dot/chooser.dot

    A variant of the selector class. Once a child is selected, it
    cannot be interrupted by higher priority siblings. As soon as the chosen child
    itself has finished it frees the chooser for an alternative selection. i.e. priorities
    only come into effect if the chooser wasn't running in the previous tick.

    .. note::
        This is the only composite in py_trees that is not a core composite in most behaviour tree implementations.
        Nonetheless, this is useful in fields like robotics, where you have to ensure that your manipulator doesn't
        drop it's payload mid-motion as soon as a higher interrupt arrives. Use this composite
        sparingly and only if you can't find another way to easily create an elegant tree composition for your task.

    Args:
        name (:obj:`str`): the composite behaviour name
        children ([:class:`~py_trees.behaviour.Behaviour`]): list of children to add
    """

    def __init__(self, name="Chooser", children=None):
        super(Chooser, self).__init__(name, children)

    def tick(self):
        """
        Run the tick behaviour for this chooser. Note that the status
        of the tick is (for now) always determined by its children, not
        by the user customised update function.

        Yields:
            :class:`~py_trees.behaviour.Behaviour`: a reference to itself or one of its children
        """
        self.logger.debug("%s.tick()" % self.__class__.__name__)
        # Required behaviour for *all* behaviours and composites is
        # for tick() to check if it isn't running and initialise
        if self.status != Status.RUNNING:
            # chooser specific initialisation
            # invalidate everything
            for child in self.children:
                child.stop(Status.INVALID)
            self.current_child = None
            # run subclass (user) initialisation
            self.initialise()
        # run any work designated by a customised instance of this class
        self.update()
        if self.current_child is not None:
            # run our child, and invalidate anyone else who may have been ticked last run
            # (bit wasteful always checking for the latter)
            for child in self.children:
                if child is self.current_child:
                    for node in self.current_child.tick():
                        yield node
                elif child.status != Status.INVALID:
                    child.stop(Status.INVALID)
        else:
            for child in self.children:
                for node in child.tick():
                    yield node
                if child.status == Status.RUNNING or child.status == Status.SUCCESS:
                    self.current_child = child
                    break
        new_status = self.current_child.status if self.current_child is not None else Status.FAILURE
        self.stop(new_status)
        yield self

##############################################################################
# Sequence
##############################################################################


class Sequence(Composite):
    """
    Sequences are the factory lines of Behaviour Trees

    .. graphviz:: dot/sequence.dot

    A sequence will progressively tick over each of its children so long as
    each child returns :data:`~py_trees.common.Status.SUCCESS`. If any child returns
    :data:`~py_trees.common.Status.FAILURE` or :data:`~py_trees.common.Status.RUNNING` the sequence will halt and the parent will adopt
    the result of this child. If it reaches the last child, it returns with
    that result regardless.

    .. note::

       The sequence halts once it sees a child is RUNNING and then returns
       the result. *It does not get stuck in the running behaviour*.

    .. seealso:: The :ref:`py-trees-demo-sequence-program` program demos a simple sequence in action.

    Args:
        name (:obj:`str`): the composite behaviour name
        children ([:class:`~py_trees.behaviour.Behaviour`]): list of children to add

    """
    def __init__(self, name="Sequence", children=None):
        super(Sequence, self).__init__(name, children)
        self.current_index = -1  # -1 indicates uninitialised

    def tick(self):
        """
        Tick over the children.

        Yields:
            :class:`~py_trees.behaviour.Behaviour`: a reference to itself or one of its children
        """
        self.logger.debug("%s.tick()" % self.__class__.__name__)
        if self.status != Status.RUNNING:
            self.logger.debug("%s.tick() [!RUNNING->resetting child index]" % self.__class__.__name__)
            # sequence specific handling
            self.current_index = 0
            for child in self.children:
                # reset the children, this helps when introspecting the tree
                if child.status != Status.INVALID:
                    child.stop(Status.INVALID)
            # subclass (user) handling
            self.initialise()
        # run any work designated by a customised instance of this class
        self.update()
        for child in itertools.islice(self.children, self.current_index, None):
            for node in child.tick():
                yield node
                if node is child and node.status != Status.SUCCESS:
                    self.status = node.status
                    yield self
                    return
            self.current_index += 1
        # At this point, all children are happy with their SUCCESS, so we should be happy too
        self.current_index -= 1  # went off the end of the list if we got to here
        self.stop(Status.SUCCESS)
        yield self

    @property
    def current_child(self):
        """
        Have to check if there's anything actually running first.

        Returns:
            :class:`~py_trees.behaviour.Behaviour`: the child that is currently running, or None
        """
        if self.current_index == -1:
            return None
        return self.children[self.current_index] if self.children else None

    def stop(self, new_status=Status.INVALID):
        """
        Stopping a sequence requires taking care of the current index. Note that
        is important to implement this here intead of terminate, so users are free
        to subclass this easily with their own terminate and not have to remember
        that they need to call this function manually.

        Args:
            new_status (:class:`~py_trees.common.Status`): the composite is transitioning to this new status
        """
        # retain information about the last running child if the new status is
        # SUCCESS or FAILURE
        if new_status == Status.INVALID:
            self.current_index = -1
        Composite.stop(self, new_status)


##############################################################################
# Parallel
##############################################################################


class Parallel(Composite):
    """
    Parallels enable a kind of concurrency

    .. graphviz:: dot/parallel.dot

    Ticks every child every time the parallel is run (a poor man's form of parallelism).

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
    def __init__(self,
                 name: str=common.Name.AUTO_GENERATED,
                 policy: common.ParallelPolicy.Base=common.ParallelPolicy.SuccessOnAll(),
                 children: typing.List[behaviour.Behaviour]=None
                 ):
        """
        Args:
            name (:obj:`str`): the composite behaviour name
            policy (:class:`~py_trees.common.ParallelPolicy`): policy to use for deciding success or otherwise
            children ([:class:`~py_trees.behaviour.Behaviour`]): list of children to add
        """
        super(Parallel, self).__init__(name, children)
        self.policy = policy

    def setup(self, **kwargs):
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

    def tick(self):
        """
        Tick over the children.

        Yields:
            :class:`~py_trees.behaviour.Behaviour`: a reference to itself or one of its children

        Raises:
            RuntimeError: if the policy configuration was invalid
        """
        self.logger.debug("%s.tick()" % self.__class__.__name__)
        self.validate_policy_configuration()
        if self.status != common.Status.RUNNING:
            self.logger.debug("%s.tick(): re-initialising" % self.__class__.__name__)
            for child in self.children:
                # reset the children, this ensures old SUCCESS/FAILURE status flags
                # don't break the synchronisation logic below
                if child.status != Status.INVALID:
                    child.stop(Status.INVALID)
            # subclass (user) handling
            self.initialise()
        # process them all first
        for child in self.children:
            if self.policy.synchronise and child.status == common.Status.SUCCESS:
                continue
            for node in child.tick():
                yield node
        new_status = common.Status.RUNNING
        if any([c.status == common.Status.FAILURE for c in self.children]):
            new_status = common.Status.FAILURE
        else:
            if type(self.policy) is common.ParallelPolicy.SuccessOnAll:
                if all([c.status == common.Status.SUCCESS for c in self.children]):
                    new_status = common.Status.SUCCESS
            elif type(self.policy) is common.ParallelPolicy.SuccessOnOne:
                if any([c.status == common.Status.SUCCESS for c in self.children]):
                    new_status = common.Status.SUCCESS
            elif type(self.policy) is common.ParallelPolicy.SuccessOnSelected:
                if all([c.status == common.Status.SUCCESS for c in self.policy.children]):
                    new_status = common.Status.SUCCESS
            else:
                raise RuntimeError("this parallel has been configured with an unrecognised policy [{}]".format(type(self.policy)))
        # this parallel may have children that are still running
        # so if the parallel itself has reached a final status, then
        # these running children need to be terminated so they don't dangle
        if new_status != common.Status.RUNNING:
            self.stop(new_status)
        self.status = new_status
        yield self

    def stop(self, new_status: common.Status=Status.INVALID):
        """
        For interrupts or any of the termination conditions, ensure that any
        running children are stopped.

        Args:
            new_status : the composite is transitioning to this new status
        """
        for child in self.children:
            if child.status == common.Status.RUNNING:
                # interrupt it (exactly as if it was interrupted by a higher priority)
                child.stop(common.Status.INVALID)

    @property
    def current_child(self):
        """
        In some cases it's clear what the current child is, in others, there
        is an ambiguity as multiple could exist. If the latter is true, it
        will return the child relevant farthest down the list.

        Returns:
            :class:`~py_trees.behaviour.Behaviour`: the child that is currently running, or None
        """
        if self.status == common.Status.INVALID:
            return None
        if self.status == common.Status.FAILURE:
            for child in self.children:
                if child.status == common.Status.FAILURE:
                    return child
            # shouldn't get here
        elif self.status == common.Status.SUCCESS:  # one, all or selected
            success_children = []
            for child in self.children:
                if child.status == common.Status.SUCCESS:
                    success_children.append(child)
            return success_children[-1]
        else:  # RUNNING
            return self.children[-1]

    def verbose_info_string(self) -> str:
        """
        Provide additional information about the underlying policy.

        Returns:
            :obj:`str`: name of the policy along with it's configuration
        """
        return str(self.policy)

    def validate_policy_configuration(self):
        """
        Policy configuration can be invalid if:

        * Policy is SuccessOnSelected and no behaviours have been specified
        * Policy is SuccessOnSelected and behaviours that are not children exist

        Raises:
            RuntimeError: if policy configuration was invalid
        """
        if type(self.policy) is common.ParallelPolicy.SuccessOnSelected:
            if not self.policy.children:
                error_message = ("policy SuccessOnSelected requires a non-empty "
                                 "selection of children [{}]".format(self.name))
                self.logger.error(error_message)
                raise RuntimeError(error_message)
            missing_children_names = [child.name for child in self.policy.children if child not in self.children]

            if missing_children_names:
                error_message = ("policy SuccessOnSelected has selected behaviours that are "
                                 "not children of this parallel {}[{}]""".format(missing_children_names, self.name))
                self.logger.error(error_message)
                raise RuntimeError(error_message)
