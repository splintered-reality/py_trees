#!/usr/bin/env python3
"""Example showing disconnected blackboards."""

import py_trees


def check_foo() -> None:
    """Read the value of a blackboard variable in a different scope."""
    blackboard = py_trees.blackboard.Client(name="Reader")
    blackboard.register_key(key="foo", access=py_trees.common.Access.READ)
    print(f"Foo: {blackboard.foo}")


blackboard = py_trees.blackboard.Client(name="Writer")
blackboard.register_key(key="foo", access=py_trees.common.Access.WRITE)
blackboard.foo = "bar"
check_foo()
