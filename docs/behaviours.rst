.. _behaviours-section:

Behaviours
==========

A :py:class:`~py_trees.behaviour.Behaviour` is the smallest element in a
behaviour tree, i.e. it is the *leaf*. Behaviours are usually representative of
either a check (am I hungry?), or an action (buy some chocolate cookies).

.. _skeleton-behaviour-include:

Skeleton
--------

Behaviours in py_trees are created by subclassing the
:class:`~py_trees.behaviour.Behaviour` class. A skeleton example:

.. literalinclude:: examples/skeleton_behaviour.py
   :language: python
   :linenos:

.. _lifecycle-section:

Lifecycle
---------

Getting a feel for how this works in action can be seen by running
the :ref:`py-trees-demo-behaviour-lifecycle-program` program (click the link
for more detail and access to the sources):

.. image:: images/lifecycle.gif

Important points to focus on:

* The :meth:`~py_trees.demos.lifecycle.Counter.initialise()` method kicks in only when the behaviour is not already running
* The parent :meth:`~py_trees.behaviour.Behaviour.tick()` method is responsible for determining when to call
  :meth:`~py_trees.demos.lifecycle.Counter.initialise()`,
  :meth:`~py_trees.demos.lifecycle.Counter.stop()` and
  :meth:`~py_trees.demos.lifecycle.Counter.terminate()` methods.
* The parent :meth:`~py_trees.behaviour.Behaviour.tick()` method always calls update()
* The :meth:`~py_trees.demos.lifecycle.Counter.update()` method is responsible for deciding the behaviour :ref:`status-section`.

.. _initialisation-section:

Initialisation
--------------

With no less than three methods used for initialisation, it can be difficult to identify where your initialisation code
needs to lurk.

.. note:: ``__init__`` should instantiate the behaviour sufficiently for offline dot graph generation

Later we'll see how we can render trees of behaviours in dot graphs. For now, it is sufficient to understand
that you need to keep this minimal enough so that you can generate dot graphs for your trees from
something like a CI server (e.g. Jenkins). This is a very useful thing to be able to do.

* No hardware connections that may not be there, e.g. usb lidars
* No middleware connections to other software that may not be there, e.g. ROS pubs/subs/services
* No need to fire up other needlessly heavy resources, e.g. heavy threads in the background

.. note:: ``setup`` handles all other one-time initialisations of resources that are required for execution

Essentially, all the things that the constructor doesn't handle - hardware connections, middleware and other heavy resources.

.. note:: ``initialise`` configures and resets the behaviour ready for (repeated) execution

Initialisation here is about getting things ready for immediate execution of a task. Some examples:

* Initialising/resetting/clearing variables
* Starting timers
* Just-in-time discovery and establishment of middleware connections
* Sending a goal to start a controller running elsewhere on the system
* ...

.. _status-section:

Status
------

The most important part of a behaviour is the determination of the behaviour's status
in the ``update()`` method. The status gets used to affect which direction
of travel is subsequently pursued through the remainder of a behaviour tree. We haven't gotten
to trees yet, but it is this which drives the decision making in a behaviour tree.

.. autoclass:: py_trees.common.Status
    :members:
    :noindex:

The ``update()`` method must return one of ``RUNNING``. ``SUCCESS`` or ``FAILURE``. A
status of ``INVALID`` is the initial default and ordinarily automatically set by other
mechansims (e.g. when a higher priority behaviour cancels the currently selected one).

.. _feedback-message-section:

Feedback Message
----------------

.. literalinclude:: ../py_trees/demos/lifecycle.py
   :language: python
   :linenos:
   :lines: 91-94

A behaviour has a naturally built in feedback message that can be
cleared in the ``initialise()`` or ``terminate()`` methods and updated
in the ``update()`` method.

.. tip:: Alter a feedback message when **significant events** occur.

The feedback message is designed to assist in notifying humans when a
significant event happens or for deciding when to log the state of
a tree. If you notify or log every tick, then you end up with a lot of
noise sorting through an abundance of data in which nothing much is
happening to find the one point where something significant occurred
that led to surprising or catastrophic behaviour.

Setting the feedback message is usually important when something
significant happens in the ``RUNNING`` state or to provide information
associated with the result (e.g. failure reason).

Example - a behaviour responsible for planning motions of a
character is in the ``RUNNING`` state for a long period of time.
Avoid updating it with a feedback message at every tick with updated plan
details. Instead, update the message whenever a significant change
occurs - e.g. when the previous plan is re-planned or pre-empted.

.. _loggers-section:

Loggers
-------

These are used throughout the demo programs. They are not intended to be
for anything heavier than debugging simple examples. This kind of logging
tends to get rather heavy and requires a lot of filtering to find the points
of change that you are interested in (see comments about the feedback
messages above).

.. _complex-example-section:

Complex Example
---------------

The :ref:`py-trees-demo-action-behaviour-program` program demonstrates
a more complicated behaviour that illustrates a few
concepts discussed above, but not present in the very simple lifecycle
:class:`~py_trees.demos.lifecycle.Counter` behaviour.

* Mocks an external process and connects to it in the ``setup`` method
* Kickstarts new goals with the external process in the ``initialise`` method
* Monitors the ongoing goal status in the ``update`` method
* Determines ``RUNNING``/``SUCCESS`` pending feedback from the external process

.. note:: A behaviour's ``update()`` method never blocks, at most it just monitors the
    progress and holds up any decision making required by a tree that is ticking the
    behaviour by setting it's status to ``RUNNING``. At the risk of being confusing, this
    is what is generally referred to as a :term:`blocking` behaviour.

.. image:: images/action.gif

