#!/usr/bin/env python3

import py_trees


class Foo(py_trees.behaviour.Behaviour):

    def __init__(self, name):
        super().__init__(name=name)
        self.blackboard = self.attach_blackboard_client(name="Foo Global")
        self.parameters = self.attach_blackboard_client(name="Foo Params", namespace="foo_parameters_")
        self.state = self.attach_blackboard_client(name="Foo State", namespace="foo_state_")

        # create a key 'foo_parameters_init' on the blackboard
        self.parameters.register_key("init", access=py_trees.common.Access.READ)
        # create a key 'foo_state_number_of_noodles' on the blackboard
        self.state.register_key("number_of_noodles", access=py_trees.common.Access.WRITE)

    def initialise(self):
        self.state.number_of_noodles = self.parameters.init

    def update(self):
        self.state.number_of_noodles += 1
        self.feedback_message = self.state.number_of_noodles
        if self.state.number_of_noodles > 5:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.RUNNING


# could equivalently do directly via the Blackboard static methods if
# not interested in tracking / visualising the application configuration
configuration = py_trees.blackboard.Client(name="App Config")
configuration.register_key("foo_parameters_init", access=py_trees.common.Access.WRITE)
configuration.foo_parameters_init = 3

foo = Foo(name="The Foo")
for i in range(1, 8):
    foo.tick_once()
    print("Number of Noodles: {}".format(foo.feedback_message))
