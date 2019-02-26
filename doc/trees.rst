.. _trees-section:

Trees
=====

.. automodule:: py_trees.trees
    :noindex:

.. _behaviour-tree-section:

The Behaviour Tree
------------------

.. autoclass:: py_trees.trees.BehaviourTree
    :noindex:

.. _skeleton-section:

Skeleton
--------

The most basic feature of the behaviour tree is it's automatic tick-tock. You can
:meth:`~py_trees.trees.BehaviourTree.tick_tock` for a specific number of iterations,
or indefinitely and use the :meth:`~py_trees.trees.BehaviourTree.interrupt` method to stop it.

.. literalinclude:: examples/skeleton_tree.py
   :language: python
   :linenos:

or create your own loop and tick at your own leisure with
the :meth:`~py_trees.trees.BehaviourTree.tick` method.

.. _pre-post-tick-handlers-section:

Pre/Post Tick Handlers
----------------------

Pre and post tick handlers can be used to perform some activity on or with the tree
immediately before and after ticking. This is mostly useful with the continuous
:meth:`~py_trees.trees.BehaviourTree.tick_tock` mechanism.

This is useful for a variety of purposes:

* logging
* doing introspection on the tree to make reports
* extracting data from the blackboard
* triggering on external conditions to modify the tree (e.g. new plan arrived)

This can be done of course, without locking since the tree won't be ticking while these
handlers run. This does however, mean that your handlers should be light. They will be
consuming time outside the regular tick period.

The :ref:`py-trees-demo-tree-stewardship-program` program demonstrates a very simple
pre-tick handler that just prints a line to stdout notifying the user of the current run.
The relevant code:

.. literalinclude:: ../py_trees/demos/stewardship.py
   :language: python
   :linenos:
   :lines: 82-92
   :caption: pre-tick-handler-function

.. literalinclude:: ../py_trees/demos/stewardship.py
   :language: python
   :linenos:
   :lines: 135-136
   :caption: pre-tick-handler-adding

.. _visitors-section:

Visitors
--------

.. automodule:: py_trees.visitors
    :noindex:

The :ref:`py-trees-demo-tree-stewardship-program` program demonstrates the two reference
visitor implementations:

* :class:`~py_trees.visitors.DebugVisitor` prints debug logging messages to stdout and
* :class:`~py_trees.visitors.SnapshotVisitor` collects runtime data to be used by visualisations

Adding visitors to a tree:

.. code-block:: python

   behaviour_tree = py_trees.trees.BehaviourTree(root)
   behaviour_tree.visitors.append(py_trees.visitors.DebugVisitor())
   snapshot_visitor = py_trees.visitors.SnapshotVisitor()
   behaviour_tree.visitors.append(snapshot_visitor)

These visitors are automatically run inside the tree's :class:`~py_trees.trees.BehaviourTree.tick` method.
The former immediately logs to screen, the latter collects information which is then used to display an
ascii tree:

.. code-block:: python

   behaviour_tree.tick()
   ascii_tree = py_trees.display.ascii_tree(
       behaviour_tree.root,
       snapshot_information=snapshot_visitor)
   )
   print(ascii_tree)



