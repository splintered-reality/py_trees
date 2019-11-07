#!/usr/bin/env python3

import py_trees


class Nested(object):
    def __init__(self):
        self.foo = None
        self.bar = None

    def __str__(self):
        return str(self.__dict__)


writer = py_trees.blackboard.Client(name="Writer")
writer.register_key(key="nested", access=py_trees.common.Access.WRITE)
reader = py_trees.blackboard.Client(name="Reader")
reader.register_key(key="nested", access=py_trees.common.Access.READ)

writer.nested = Nested()
writer.nested.foo = "I am foo"
writer.nested.bar = "I am bar"

foo = reader.nested.foo
print(writer)
print(reader)
