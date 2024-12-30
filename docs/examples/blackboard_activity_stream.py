#!/usr/bin/env python3
"""Example showing how to display the blackboard activity stream."""

import py_trees

py_trees.blackboard.Blackboard.enable_activity_stream(maximum_size=100)
reader = py_trees.blackboard.Client(name="Reader")
reader.register_key(key="foo", access=py_trees.common.Access.READ)
writer = py_trees.blackboard.Client(name="Writer")
writer.register_key(key="foo", access=py_trees.common.Access.WRITE)
writer.foo = "bar"
writer.foo = "foobar"
unused_result = reader.foo
print(py_trees.display.unicode_blackboard_activity_stream())
assert py_trees.blackboard.Blackboard.activity_stream is not None
py_trees.blackboard.Blackboard.activity_stream.clear()
