The Building Blocks
===================

Behaviours
----------

**What is it?**

A :py:class:`~py_trees.behaviour.Behaviour` is the smallest element in a
behaviour tree, i.e. it is the *leaf*. Behaviours are usually representative of
either a check (am I hungry?), or an action (buy some chocolate cookies).

**How it Works**

When a tree is traversed, individual behaviours execute a small chunk of code
relevant to the check, or action they are responsible for managing (triggering,
monitoring, getting the result).  The result of that execution is a
:py:class:`~py_trees.common.Status` value (``INVALID``/``RUNNING``/``SUCCESS``/``FAILURE``)
that is used to affect the continuing direction of traversal across the tree.
This is the decision making part of the process.

Execution of a behaviour is referred to as :term:`ticking` the behaviour.

.. note:: Behaviours must be non-blocking.

To support the concept of :term:`ticking`, the execution part of a behaviour
should be *non-blocking*. Checking values, triggering an external action,
checking on the feedback/result from that action are all good.
Sitting there and looping inside the execution method while waiting for the robot
to rotate pi/2 radians is not. Instead, prefer the following flow over multiple ticks
(executions):

1. Trigger the rotation -> ``RUNNING``
2. Monitor the rotation's feedback -> ``RUNNING``
3. Monitor the rotation's feedback -> ``RUNNING``
4. Monitor the rotation's feedback -> ``RUNNING``
5. Rotation stopped, get the result -> ``SUCCESS/FAILURE``

At the risk of being confusing, a behaviour that takes several ticks to progress
from ``RUNNING`` to ``SUCCESS/FAILURE`` as show above is referred to as a :term:`blocking`
behaviour.

**The LifeCycle**

Easy to see via the ``demo_behavour`` program.

.. literalinclude:: ../scripts/demo_behaviour
   :language: python
   :linenos:

with output:

.. code-block:: bash

   [DEBUG] Demo Behaviours : Counter [Counter::__init__()]
   [DEBUG] Demo Behaviours : Counter [Counter::setup()]
   [DEBUG] Demo Behaviours : Counter [Behaviour.tick()]
   [DEBUG] Demo Behaviours : Counter [Counter::initialise()]
   [DEBUG] Demo Behaviours : Counter [Counter.update()][Status.INVALID->Status.RUNNING][count: 1]
   [DEBUG] Demo Behaviours : Counter [Behaviour.tick()]
   [DEBUG] Demo Behaviours : Counter [Counter.update()][Status.RUNNING->Status.RUNNING][count: 2]
   [DEBUG] Demo Behaviours : Counter [Behaviour.tick()]
   [DEBUG] Demo Behaviours : Counter [Counter.update()][Status.RUNNING->Status.SUCCESS][count: 3]
   [DEBUG] Demo Behaviours : Counter [Behaviour.stop()]
   [DEBUG] Demo Behaviours : Counter [Counter.terminate()][Status.RUNNING->Status.SUCCESS]
   [DEBUG] Demo Behaviours : Counter [Behaviour.tick()]
   [DEBUG] Demo Behaviours : Counter [Counter::initialise()]
   [DEBUG] Demo Behaviours : Counter [Counter.update()][Status.SUCCESS->Status.RUNNING][count: 1]
   [DEBUG] Demo Behaviours : Counter [Behaviour.tick()]
   [DEBUG] Demo Behaviours : Counter [Counter.update()][Status.RUNNING->Status.RUNNING][count: 2]
   [DEBUG] Demo Behaviours : Counter [Behaviour.tick()]
   [DEBUG] Demo Behaviours : Counter [Counter.update()][Status.RUNNING->Status.SUCCESS][count: 3]
   [DEBUG] Demo Behaviours : Counter [Behaviour.stop()]
   [DEBUG] Demo Behaviours : Counter [Counter.terminate()][Status.RUNNING->Status.SUCCESS]
   [DEBUG] Demo Behaviours : Counter [Behaviour.tick()]
   [DEBUG] Demo Behaviours : Counter [Counter::initialise()]
   [DEBUG] Demo Behaviours : Counter [Counter.update()][Status.SUCCESS->Status.RUNNING][count: 1]

Above, the ``Behaviour.tick()`` and ``Behaviour.stop()`` are parent level methods that manage the
running of the behaviour. All other methods are reserved for custom code specific to the
behaviour.

Sequences
---------

**Sequences are the factory lines of Behaviour Trees**

A :py:class:`~py_trees.composites.Sequence` is the first of the two fundamental
composite behaviours. They are themseleves a behaviour, but are also responsible
for a family of behaviour children. A sequence will progressively tick over each
of its children so long as each child returns ``SUCCESS``. If any child returns
FAILURE or RUNNING the sequence will halt and the parent will adopt the result of this child.
If it reaches the last child, it returns with that result regardless.

.. note::

   The sequence halts once it sees a child is RUNNING and then returns the result -> *it does
   not get stuck in the running behaviour*.

Selectors
---------

**Selectors are the Decision Makers**

A :py:class:`~py_trees.composites.Selector` is the second of the two fundamental composite
behaviours. A selector executes each of its child behaviours in turn until one of them
succeeds (at which point it itself returns ``SUCCESS``) or it runs out of children
(at which point it itself returns ``FAILURE``). We usually refer to selecting children as a
means of *choosing between priorities*. Each child and its subtree represent a lower or
higher priority path.

.. todo:: image representing priorities from high to low (left to right)

.. note::

   Switching from a low -> high priority branch causes a `stop(INVALID)` signal to be sent to the previously
   executing low priority branch. This signal will percolate down that child's own subtree. Behaviours
   should make sure that they catch this and ‘destruct’ appropriately.

Data Sharing
------------

.. todo:: blackboards

Tree Containers
---------------

You stuff an assembled tree into these containers - the container takes care of alot of tree handling for you.
The :py:class:`~py_trees.trees.BehaviourTree` handles logging, insertions, tick_tock.

.. todo:: Example Code - maybe put this in the class docs itself.

The :py:class:`~py_trees.trees.ROSBehaviourTree` subclasses the BehaviourTree and additionally
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