#!/usr/bin/env python3
"""Example showing how to use objects to create nested blackboard keys."""

import py_trees


class Nested(object):
    """Simple object that contains a few attributes."""

    def __init__(self) -> None:
        self.foo: str | None = None
        self.bar: str | None = None

    def __str__(self) -> str:
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
