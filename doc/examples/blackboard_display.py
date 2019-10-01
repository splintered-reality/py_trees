#!/usr/bin/env python3

import py_trees

writer = py_trees.blackboard.BlackboardClient(
    name="Writer",
    write={"foo", "bar", "dude", "dudette"}
)
reader = py_trees.blackboard.BlackboardClient(
    name="Reader",
    read={"foo", "bar"}
)
writer.foo = "foo"
writer.bar = "bar"
writer.dude = "bob"

# all key-value pairs
print(py_trees.display.unicode_blackboard())
# various filtered views
print(py_trees.display.unicode_blackboard(key_filter={"foo"}))
print(py_trees.display.unicode_blackboard(regex_filter="dud*"))
print(py_trees.display.unicode_blackboard(client_filter={reader.unique_identifier}))
# list the clients associated with each key
print(py_trees.display.unicode_blackboard(display_only_key_metadata=True))
