#!/usr/bin/env python3

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
py_trees.blackboard.Blackboard.activity_stream.clear()
