#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/yujinrobot/gopher_crazy_hospital/license/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
.. module:: composites
   :platform: Unix
   :synopsis: This module defines standard composites for use in py_trees.

The most important ones are the selector and sequence, but some more esoteric
versions are also included here.

----

"""

##############################################################################
# Imports
##############################################################################

from __future__ import absolute_import

import itertools

from . import common
from . import logging
from .behaviour import Behaviour
from .common import Status
from . import meta

##############################################################################
# Composites
##############################################################################


class Composite(Behaviour):
    """ A node in a behavior tree that composites behaviours """
    def __init__(self, name="", children=None, *args, **kwargs):
        super(Composite, self).__init__(name, *args, **kwargs)
        if children is not None:
            map(self.add_child, children)
        else:
            self.children = []
        self.logger = logging.get_logger("Composite")

    ############################################
    # Worker Overrides
    ############################################
    def setup(self, timeout):
        """
        Override this function to do any post-constructor setup that is necessary
        for a runtime. A good example are any of the ros wait_for methods which
        will block while looking for a ros master/topics/services.

        This is best done here rather than in the constructor so that trees can
        be instantiated on the fly without any runtime requirements to produce
        visualisations such as dot graphs.

        :param double timeout: time to wait (0.0 is blocking forever)
        :returns: whether it timed out waiting for the server or not.
        :rtype: boolean
        """
        self.logger.debug("  %s [Composite.setup()])" % (self.name))
        result = True
        for child in self.children:
            new_result = child.setup(timeout)
            if new_result is None:
                # replace with py_trees exception!
                self.logger.error("  %s [Composite.setup()]['%s'.setup() returned None (must be True||False)" % (self.name, child.name))
            result = result and new_result
        return result

    def stop(self, new_status=Status.INVALID):
        """
        This updates all the children. We generally have two use cases here - whenever the composite has
        gone to a recognised state (i.e. FAILURE or SUCCESS), or when a higher level parent calls on
        it to truly stop (INVALID). We only really want to update everything below if we are in
        the second use case. The first use case will subehaviours will have already handled themselves.
        """
        self.logger.debug("  %s [Composite.stop()][%s->%s]" % (self.name, self.status, new_status))
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
        Recursive function to extract the last running node of the tree. Returns the
        tip function of the current child of this composite.

        """
        return self.current_child.tip() if self.current_child is not None else None

    ############################################
    # Children
    ############################################

    def add_child(self, child):
        """
        Adds a child.

        :param child: child to add to the tree
        :type child: instance or descendant of :class:`Behaviour <py_trees.behaviours.Behaviour>`
        :return: a unique id for this child (store and use to form remove requests)
        """
        assert isinstance(child, Behaviour), "children must be behaviours, but you passed in %s" % type(child)
        self.children.append(child)
        child.parent = self
        return child.id

    def add_children(self, children):
        """
        Adds a child.

        :param Behaviour[] children: children to append to the composite.
        """
        for child in children:
            assert isinstance(child, Behaviour), "children must be behaviours, but you passed in %s" % type(child)
            self.children.append(child)
            child.parent = self

    def remove_child(self, child):
        """
        Remove the child behaviour from this composite.

        :param child: child to remove from the tree
        :type child: instance or descendant of :class:`Behaviour <py_trees.behaviours.Behaviour>`

        .. todo:: error handling for when child is not in this list (what error is thrown?)
        """
        if child.status == Status.RUNNING:
            child.stop(Status.INVALID)
        index = self.children.index(child)
        self.children.remove(child)
        return index

    def remove_all_children(self):
        """
        Remove all children. Makes sure to stop each child if necessary.
        """
        for child in self.children:
            if child.status == Status.RUNNING:
                child.stop(Status.INVALID)
        # makes sure to delete it for this class and all references to it
        #   http://stackoverflow.com/questions/850795/clearing-python-lists
        del self.children[:]

    def replace_child(self, child, replacement):
        """
        Replace the child behaviour.

        :param child: child to remove from the tree
        :type child: instance or descendant of :class:`Behaviour <py_trees.behaviours.Behaviour>`
        :param replacement: child to insert
        :type replacement: instance or descendant of :class:`Behaviour <py_trees.behaviours.Behaviour>`
        """
        if child.status == Status.RUNNING:
            child.stop(Status.INVALID)
        index = self.children.index(child)
        self.logger.debug("  %s [replace_child()][%s->%s]" % (self.name, child.name, replacement.name))
        self.children[index] = replacement

    def remove_child_by_id(self, child_id):
        child = next((c for c in self.children if c.id == child_id), None)
        if child is not None:
            if child.status == Status.RUNNING:
                child.stop(Status.INVALID)
            self.children.remove(child)
        else:
            raise IndexError('child was not found with the specified id [%s]' % child_id)

    def prepend_child(self, child):
        self.children.insert(0, child)
        child.parent = self
        return child.id

    def insert_child(self, child, index):
        self.children.insert(index, child)
        child.parent = self
        return child.id

##############################################################################
# Selector
##############################################################################


class Selector(Composite):
    """
    Runs all of its child behaviours in sequence until one succeeds.
    If a node after than the one selected was previously running, then call
    stop() on it.
    """

    def __init__(self, name="Selector", children=None, *args, **kwargs):
        super(Selector, self).__init__(name, children, *args, **kwargs)
        self.current_child = None
        self.logger = logging.get_logger("Selector ")

    def tick(self):
        """
        Run the tick behaviour for this selector. Note that the status
        of the tick is (for now) always determined by its children, not
        by the user customised update function.

        For now, the update function just provides a place where classes
        that inherit this one can do some work *before* running the children.
        The return status is ignored.

        .. todo::

           Need a strict policy about handling the return status.
           Should it abort if that returns success or fail, should it be ignored,
           should it be done before/after children, should it be open to configuration?
        """
        self.logger.debug("  %s [Selector.tick()]" % self.name)
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
                        if (previous is not None) and (previous != self.current_child) and (previous.status != Status.INVALID):
                            previous.stop(Status.INVALID)
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
        is important to implement this here intead of terminate, so users are free
        to subclass this easily with their own terminate and not have to remember
        that they need to call this function manually.
        """
        # retain information about the last running child if the new status is
        # SUCCESS or FAILURE
        if new_status == Status.INVALID:
            self.current_child = None
        Composite.stop(self, new_status)

    def __repr__(self):
        s = "Name       : %s\n" % self.name
        s += "  Status  : %s\n" % self.status
        s += "  Current : %s\n" % (self.current_child.name if self.current_child is not None else "none")
        s += "  Children: %s\n" % [child.name for child in self.children]

##############################################################################
# Sequence
##############################################################################


class Sequence(Composite):
    """
    Runs all of its children in sequence until all succeed
    """

    def __init__(self, name="Sequence", children=None, *args, **kwargs):
        super(Sequence, self).__init__(name, children, *args, **kwargs)
        self.current_index = -1  # -1 indicates uninitialised
        self.logger = logging.get_logger("Sequence ")

    def tick(self):
        if self.status != Status.RUNNING:
            # sequence specific handling
            self.current_index = 0
            # subclass (user) handling
            self.initialise()
        self.logger.debug("  %s [tick()]" % self.name)
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
        if self.current_index == -1:
            return None
        return self.children[self.current_index] if self.children else None

    def stop(self, new_status=Status.INVALID):
        """
        Stopping a sequence requires taking care of the current index. Note that
        is important to implement this here intead of terminate, so users are free
        to subclass this easily with their own terminate and not have to remember
        that they need to call this function manually.
        """
        self.logger.debug("  %s [stop()][%s->%s]" % (self.name, self.status, new_status))
        # retain information about the last running child if the new status is
        # SUCCESS or FAILURE
        if new_status == Status.INVALID:
            self.current_index = -1
        Composite.stop(self, new_status)


@meta.oneshot
class OneshotSequence(Sequence):
    pass

##############################################################################
# Paralllel
##############################################################################


class Parallel(Composite):
    """
    Ticks every child every time the parallel is run (a poor man's form of paralellism).

    Will return :py:data:`~py_trees.common.Status.RUNNING` if any one of them is still
    running, :py:data:`~py_trees.common.Status.SUCCESS` if they all succeed, or
    :py:data:`~py_trees.common.Status.FAILURE` if any single one of them fails.
    """
    def __init__(self, name="Parallel", policy=common.ParallelPolicy.SUCCESS_ON_ALL, children=None, *args, **kwargs):
        super(Parallel, self).__init__(name, children, *args, **kwargs)
        self.logger = logging.get_logger(name)
        self.policy = policy

    def tick(self):
        if self.status != Status.RUNNING:
            # subclass (user) handling
            self.initialise()
        self.logger.debug("  %s [tick()]" % self.name)
        # process them all first
        for child in self.children:
            for node in child.tick():
                yield node
        # new_status = Status.SUCCESS if self.policy == common.ParallelPolicy.SUCCESS_ON_ALL else Status.RUNNING
        new_status = Status.RUNNING
        if any([c.status == Status.FAILURE for c in self.children]):
            new_status = Status.FAILURE
        else:
            if self.policy == common.ParallelPolicy.SUCCESS_ON_ALL:
                if all([c.status == Status.SUCCESS for c in self.children]):
                    new_status = Status.SUCCESS
            elif self.policy == common.ParallelPolicy.SUCCESS_ON_ONE:
                if any([c.status == Status.SUCCESS for c in self.children]):
                    new_status = Status.SUCCESS
        # special case composite - this parallel may have children that are still running
        # so if the parallel itself has reached a final status, then these running children
        # need to be made aware of it too
        if new_status != Status.RUNNING:
            for child in self.children:
                if child.status == Status.RUNNING:
                    # yes, INVALID is right, we are interrupting it. Couldn't use new_status
                    # anyway, because composites only pass along stop to their children on
                    # INVALID. That policy is so that we don't end up virallly failing behaviours
                    # that already succeeded.
                    child.stop(new_status)
            self.stop(new_status)
        self.status = new_status
        yield self

    @property
    def current_child(self):
        if self.status == Status.INVALID:
            return None
        if self.status == Status.FAILURE:
            for child in self.children:
                if child.status == Status.FAILURE:
                    return child
            # shouldn't get here
        elif self.status == Status.SUCCESS and self.policy == common.ParallelPolicy.SUCCESS_ON_ONE:
            for child in self.children:
                if child.status == Status.SUCCESS:
                    return child
        else:
            return self.children[-1]
