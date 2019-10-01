#!/usr/bin/env python3

import py_trees


class Nested(object):
    def __init__(self):
        self.foo = None
        self.bar = None

    def __str__(self):
        return str(self.__dict__)


writer = py_trees.blackboard.BlackboardClient(
    name="Writer",
    write={"nested"}
)
reader = py_trees.blackboard.BlackboardClient(
    name="Reader",
    read={"nested"}
)
writer.nested = Nested()
writer.nested.foo = "foo"
writer.nested.bar = "bar"

foo = reader.nested.foo
print(writer)
print(reader)
