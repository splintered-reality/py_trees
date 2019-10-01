#!/usr/bin/env python3

import py_trees

py_trees.blackboard.Blackboard.enable_activity_stream(maximum_size=100)
blackboard_reader = py_trees.blackboard.BlackboardClient(name="Reader", read={"foo"})
blackboard_writer = py_trees.blackboard.BlackboardClient(name="Writer", write={"foo"})
blackboard_writer.foo = "bar"
blackboard_writer.foo = "foobar"
unused_result = blackboard_reader.foo
print(py_trees.display.unicode_blackboard_activity_stream())
py_trees.blackboard.Blackboard.activity_stream.clear()
