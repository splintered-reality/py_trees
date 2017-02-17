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

Data Sharing
------------

.. todo:: blackboards

Tree Containers
---------------

You stuff an assembled tree into these containers - the container takes care of alot of tree handling for you.
The :py:class:`~py_trees.trees.BehaviourTree` handles logging, insertions, tick_tock.

.. todo:: Example Code - maybe put this in the class docs itself.

The :py:class:`~py_trees.trees.ros.BehaviourTree` subclasses the BehaviourTree and additionally
takes care of all the handles that go out to the rqt monitoring program program.

Tree Management
---------------

.. todo:: Visitors and Pre/Post Tick Handlers

Visualisations
--------------

Dot Graphs
^^^^^^^^^^

You can render trees into dot/png/svg files simply by calling the :py:func:`~py_trees.display.render_dot_tree`
function. There is also an ascii version.

.. code-block:: python

   root = py_trees.Sequence(name="Sequence")
   guard = py_trees.behaviours.Success("Guard")
   periodic_success = py_trees.behaviours.Periodic("Periodic", 3)
   finisher = py_trees.behaviours.Success("Finisher")
   root.add_child(guard)
   root.add_child(periodic_success)
   root.add_child(finisher)
   py_trees.display.render_dot_tree(root)

To enable quick generation of dotgraphs for your *subtrees*, use a class method inside your root class, e.g.

.. code-block:: python

   @classmethod
   def render_dot_tree(cls):
       root = cls()
       py_trees.display.render_dot_tree(root)

Online/Offline Monitoring
^^^^^^^^^^^^^^^^^^^^^^^^^

.. todo:: RQT Py Trees program