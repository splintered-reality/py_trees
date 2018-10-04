# doc/examples/skeleton_behaviour.py

import py_trees
import random


class Foo(py_trees.Behaviour):
    def __init__(self, name):
        """
        Minimal one-time initialisation. A good rule of thumb is
        to only include the initialisation relevant for being able
        to insert this behaviour in a tree for offline rendering to
        dot graphs.

        Other one-time initialisation requirements should be met via
        the setup() method.
        """
        super(Foo, self).__init__(name)

    def setup(self, timeout):
        """
        When is this called?
          This function should be either manually called by your program
          or indirectly called by a parent behaviour when it's own setup
          method has been called.

          If you have vital initialisation here, a useful design pattern
          is to put a guard in your initialise() function to barf the
          first time your behaviour is ticked if setup has not been
          called/succeeded.

        What to do here?
          Delayed one-time initialisation that would otherwise interfere
          with offline rendering of this behaviour in a tree to dot graph.
          Good examples include:
          - Hardware or driver initialisation
          - Middleware initialisation (e.g. ROS pubs/subs/services)
        """
        self.logger.debug("  %s [Foo::setup()]" % self.name)

    def initialise(self):
        """
        When is this called?
          The first time your behaviour is ticked and anytime the
          status is not RUNNING thereafter.

        What to do here?
          Any initialisation you need before putting your behaviour
          to work.
        """
        self.logger.debug("  %s [Foo::initialise()]" % self.name)

    def update(self):
        """
        When is this called?
          Every time your behaviour is ticked.

        What to do here?
          - Triggering, checking, monitoring. Anything...but do not block!
          - Set a feedback message
          - return a py_trees.Status.[RUNNING, SUCCESS, FAILURE]
        """
        self.logger.debug("  %s [Foo::update()]" % self.name)
        ready_to_make_a_decision = random.choice([True, False])
        decision = random.choice([True, False])
        if not ready_to_make_a_decision:
            return py_trees.Status.RUNNING
        elif decision:
            self.feedback_message = "We are not bar!"
            return py_trees.Status.SUCCESS
        else:
            self.feedback_message = "Uh oh"
            return py_trees.Status.FAILURE

    def terminate(self, new_status):
        """
        When is this called?
           Whenever your behaviour switches to a non-running state.
            - SUCCESS || FAILURE : your behaviour's work cycle has finished
            - INVALID : a higher priority branch has interrupted, or shutting down
        """
        self.logger.debug("  %s [Foo::terminate().terminate()][%s->%s]" % (self.name, self.status, new_status))
