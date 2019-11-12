#!/usr/bin/env python3

import py_trees

blackboard = py_trees.blackboard.Client(name="Global")
parameters = py_trees.blackboard.Client(name="Parameters", namespace="parameters")

blackboard.register_key(key="foo", access=py_trees.common.Access.WRITE)
blackboard.register_key(key="/bar", access=py_trees.common.Access.WRITE)
blackboard.register_key(key="/parameters/default_speed", access=py_trees.common.Access.WRITE)
parameters.register_key(key="aggressive_speed", access=py_trees.common.Access.WRITE)

blackboard.foo = "foo"
blackboard.bar = "bar"
blackboard.parameters.default_speed = 20.0
parameters.aggressive_speed = 60.0

miss_daisy = blackboard.parameters.default_speed
van_diesel = parameters.aggressive_speed

print(blackboard)
print(parameters)
