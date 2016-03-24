Examples
========

A Skeleton Behaviour
--------------------

.. code-block:: python

   class Foo(py_trees.Behaviour):

       def __init__(self, name)
           super(Foo, self).__init__(name)
           # basic variable construction here
           # leave any ros init_node dependent setup to the setup()
           # function so we can generate dot graphs offline
           #

       def setup(self, timeout)
           self.logger.debug("  %s [Foo::setup()]" % self.name)
           #
           # When is this called?
           #   This is not automatically called, your tree or
           #   your program should call the setup function after construction
           #   to make sure it gets run. A useful pattern is to set a flag so
           #   that your initialise function can barf the first time your
           #   behaviour is ticked.
           #
           # What to do here?
           #   Do most of your ros setup here, candidates are:
           #   - publishers, subscribers, services, actions with private names
           #   - wait_for_xxx calls
           #

       def initialise(self)
           self.logger.debug("  %s [Foo::initialise()]" % self.name)
           #
           # When is this called?
           #   The first time your behaviour is ticked and anytime the
           #   status is not RUNNING thereafter.
           #
           # What to do here?
           #   Any clearing, initialisation you need before putting your behaviour
           #   though a period of work.
           #

       def update(self)
           self.logger.debug("  %s [Foo::update()]" % self.name)
           #
           # When is this called?
           #   Every time your behaviour is ticked.
           #
           # What to do here?
           #   Triggering, checking, monitring. Anything...but do not block!
           #   Must return a py_trees.Status.[RUNNING, SUCCESS, FAILURE]
           #

       def terminate(self, new_status)
           self.logger.debug("  %s [Foo::terminate()]" % self.name)
           #
           # When is this called?
           #
           #    Whenever your behaviour changes state.
           #     - SUCCESS || FAILURE : your behaviour's work cycle has finished
           #     - INVALID : a higher priority branch has interrupted, or shutting down
           #



A Skeleton Sequence
-------------------

A Skeleton Selector
-------------------


A Demo Tree
-----------

.. graphviz:: dot/demo_tree.dot

Run the `py_trees/scripts/demo_tree` script.

.. todo:: link to the file from here
.. todo:: point out some important highlight points of the script, e.g. step 4 'such and such happens'
.. todo:: maybe put these pointers in as descriptive comments in the tree itself.

A Demo ROS Tree
---------------

Making a SubTree
----------------

Offline DotViz Generation
-------------------------



