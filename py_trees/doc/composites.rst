Composites
==========

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

The subsections below introduce each composite briefly. For a full listing of each
composite's methods, visit the :ref:`py-trees-composites-module` module api documentation.

.. tip:: First time through, make sure to follow the link through to relevant demo programs.

Sequence
--------

.. autoclass:: py_trees.composites.Sequence
    :noindex:

Selector
--------

.. autoclass:: py_trees.composites.Selector
    :noindex:

Chooser
-------

.. autoclass:: py_trees.composites.Chooser
    :noindex:

Parallel
--------

.. autoclass:: py_trees.composites.Parallel
    :noindex:
