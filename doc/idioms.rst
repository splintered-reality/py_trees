Idioms
======

.. automodule:: py_trees.idioms
    :noindex:

Common decision making patterns can often be realised using a specific
combination of fundamental behaviours and the blackboard. Even if this
somewhat verbosely populates the tree, this is preferable to creating
new composites types or overriding existing composites since this will
increase tree logic complexity and/or bury details under the hood (both
of which add an exponential cost to introspection/visualisation).

In this package these patterns will be referred to as **PyTree Idioms**
and in this module you will find convenience functions that assist in
creating them.

The subsections below introduce each composite briefly. For a full listing of each
composite's methods, visit the :ref:`py-trees-idioms-module` module api documentation.

.. tip:: First time through, make sure to follow the link through to relevant demo programs.

Oneshot
-------

.. automethod:: py_trees.idioms.oneshot
    :noindex:
