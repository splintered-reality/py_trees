#!/usr/bin/env python
#
# License: BSD
#   https://raw.githubusercontent.com/splintered-reality/py_trees/devel/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
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

* :class:`~py_trees.composites.Selector`: select a child to execute based on cascading priorities
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

import itertools
import typing

from . import behaviour
from . import common

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
                 name: typing.Union[str, common.Name]=common.Name.AUTO_GENERATED,
                 children: typing.List[behaviour.Behaviour]=None
                 ):
        super(Composite, self).__init__(name)
        if children is not None:
            for child in children:
                self.add_child(child)
        else:
            self.children = []
        self.current_child = None

    ############################################
    # Worker Overrides
    ############################################

    def stop(self, new_status=common.Status.INVALID):
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
        # priority interrupted
        if new_status == common.Status.INVALID:
            self.current_child = None
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
            TypeError: if the child is not an instance of :class:`~py_trees.behaviour.Behaviour`
            RuntimeError: if the child already has a parent

        Returns:
            uuid.UUID: unique id of the child
        """
        if not isinstance(child, behaviour.Behaviour):
            raise TypeError("children must be behaviours, but you passed in {}".format(type(child)))
        self.children.append(child)
        if child.parent is not None:
            raise RuntimeError("behaviour '{}' already has parent '{}'".format(child.name, child.parent.name))
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
        return self

    def remove_child(self, child):
        """
        Remove the child behaviour from this composite.

        Args:
            child (:class:`~py_trees.behaviour.Behaviour`): child to delete

        Returns:
            :obj:`int`: index of the child that was removed

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

    def remove_all_children(self):
        """
        Remove all children. Makes sure to stop each child if necessary.
        """
        self.current_child = None
        for child in self.children:
            if child.status == common.Status.RUNNING:
                child.stop(common.Status.INVALID)
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
        self.logger.debug("%s.replace_child()[%s->%s]" % (self.__class__.__name__, child.name, replacement.name))
        child_index = self.children.index(child)
        self.remove_child(child)
        self.insert_child(replacement, child_index)
        child.parent = None

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
            self.remove_child(child)
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
    Selectors are the decision makers.

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

    .. seealso:: The :ref:`py-trees-demo-selector-program` program demos higher priority switching under a selector.

    Args:
        name (:obj:`str`): the composite behaviour name
        memory (:obj:`bool`): if :data:`~py_trees.common.Status.RUNNING` on the previous tick, resume with the :data:`~py_trees.common.Status.RUNNING` child
        children ([:class:`~py_trees.behaviour.Behaviour`]): list of children to add
    """

    def __init__(self, name="Selector", memory=False, children=None):
        super(Selector, self).__init__(name, children)
        self.memory = memory

    def tick(self):
        """
        Run the tick behaviour for this selector. Note that the status
        of the tick is always determined by its children, not
        by the user customised update function.

        Yields:
            :class:`~py_trees.behaviour.Behaviour`: a reference to itself or one of its children
        """
        self.logger.debug("%s.tick()" % self.__class__.__name__)
        # initialise
        if self.status != common.Status.RUNNING:
            # selector specific initialisation - leave initialise() free for users to
            # re-implement without having to make calls to super()
            self.logger.debug("%s.tick() [!RUNNING->reset current_child]" % self.__class__.__name__)
            self.current_child = self.children[0] if self.children else None

            # reset the children - don't need to worry since they will be handled
            # a) prior to a remembered starting point, or
            # b) invalidated by a higher level priority

            # user specific initialisation
            self.initialise()

        # customised work
        self.update()

        # nothing to do
        if not self.children:
            self.current_child = None
            self.stop(common.Status.FAILURE)
            yield self
            return

        # starting point
        if self.memory:
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
                    if node.status == common.Status.RUNNING or node.status == common.Status.SUCCESS:
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

    def stop(self, new_status=common.Status.INVALID):
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
        if new_status == common.Status.INVALID:
            self.current_child = None
        Composite.stop(self, new_status)

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
        name: the composite behaviour name
        memory: if :data:`~py_trees.common.Status.RUNNING` on the previous tick, resume with the :data:`~py_trees.common.Status.RUNNING` child
        children: list of children to add

    """
    def __init__(
        self,
        name: str="Sequence",
        memory: bool=True,
        children: typing.List[behaviour.Behaviour]=None
    ):
        super(Sequence, self).__init__(name, children)
        self.memory = memory

    def tick(self):
        """
        Tick over the children.

        Yields:
            :class:`~py_trees.behaviour.Behaviour`: a reference to itself or one of its children
        """
        self.logger.debug("%s.tick()" % self.__class__.__name__)

        # initialise
        index = 0
        if self.status != common.Status.RUNNING or not self.memory:
            self.current_child = self.children[0] if self.children else None
            for child in self.children:
                if child.status != common.Status.INVALID:
                    child.stop(common.Status.INVALID)
            # user specific initialisation
            self.initialise()
        else:  # self.memory is True and status is RUNNING
            index = self.children.index(self.current_child)

        # customised work
        self.update()

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
                 name: typing.Union[str, common.Name]=common.Name.AUTO_GENERATED,
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
            failed_child = next(child for child in self.children if child.status == common.Status.FAILURE)
            self.current_child = failed_child
            new_status = common.Status.FAILURE
        except StopIteration:
            if type(self.policy) is common.ParallelPolicy.SuccessOnAll:
                if all([c.status == common.Status.SUCCESS for c in self.children]):
                    new_status = common.Status.SUCCESS
                    self.current_child = self.children[-1]
            elif type(self.policy) is common.ParallelPolicy.SuccessOnOne:
                successful = [child for child in self.children if child.status == common.Status.SUCCESS]
                if successful:
                    new_status = common.Status.SUCCESS
                    self.current_child = successful[-1]
            elif type(self.policy) is common.ParallelPolicy.SuccessOnSelected:
                if all([c.status == common.Status.SUCCESS for c in self.policy.children]):
                    new_status = common.Status.SUCCESS
                    self.current_child = self.policy.children[-1]
            else:
                raise RuntimeError("this parallel has been configured with an unrecognised policy [{}]".format(type(self.policy)))
        # this parallel may have children that are still running
        # so if the parallel itself has reached a final status, then
        # these running children need to be terminated so they don't dangle
        if new_status != common.Status.RUNNING:
            self.stop(new_status)
        self.status = new_status
        yield self

    def stop(self, new_status: common.Status=common.Status.INVALID):
        """
        For interrupts or any of the termination conditions, ensure that any
        running children are stopped.

        Args:
            new_status : the composite is transitioning to this new status
        """
        # clean up dangling (running) children
        for child in self.children:
            if child.status == common.Status.RUNNING:
                # interrupt it exactly as if it was interrupted by a higher priority
                child.stop(common.Status.INVALID)
        # only nec. thing here is to make sure the status gets set to INVALID if
        # it was a higher priority interrupt (new_status == INVALID)
        Composite.stop(self, new_status)

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
