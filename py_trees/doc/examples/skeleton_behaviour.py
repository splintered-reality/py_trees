# doc/examples/skeleton_behaviour.py

import py_trees
import random


class Foo(py_trees.Behaviour):
    def __init__(self, name):
        """
        Basic variable construction here
        leave any ros init_node dependent setup to the setup()
        function so we can generate dot graphs offline
        """
        super(Foo, self).__init__(name)

    def setup(self, timeout):
        """
        When is this called?
          This is not automatically called, your tree or
          your program should call the setup function after construction
          to make sure it gets run. A useful pattern is to set a flag so
          that your initialise function can barf the first time your
          behaviour is ticked.

        What to do here?
          Do most of your ros setup here, candidates are:
          - publishers, subscribers, services, actions with private names
          - wait_for_xxx calls
        """
        self.logger.debug("  %s [Foo::setup()]" % self.name)

    def initialise(self):
        """
        When is this called?
          The first time your behaviour is ticked and anytime the
          status is not RUNNING thereafter.

        What to do here?
          Any clearing, initialisation you need before putting your behaviour
          though a period of work.
        """
        self.logger.debug("  %s [Foo::initialise()]" % self.name)

    def update(self):
        """
        When is this called?
          Every time your behaviour is ticked.

        What to do here?
          - Triggering, checking, monitring. Anything...but do not block!
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
           Whenever your behaviour changes state.
            - SUCCESS || FAILURE : your behaviour's work cycle has finished
            - INVALID : a higher priority branch has interrupted, or shutting down
        """
        self.logger.debug("  %s [Foo::terminate().terminate()][%s->%s]" % (self.name, self.status, new_status))
