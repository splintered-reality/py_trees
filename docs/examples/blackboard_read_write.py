#!/usr/bin/env python3
"""Example demonstrating basic read/write operations on the blackboard."""

import py_trees

blackboard = py_trees.blackboard.Client(name="Client")
blackboard.register_key(key="foo", access=py_trees.common.Access.WRITE)
blackboard.register_key(key="bar", access=py_trees.common.Access.READ)
blackboard.foo = "foo"
print(blackboard)
