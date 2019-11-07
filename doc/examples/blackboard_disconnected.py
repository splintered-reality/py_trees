#!/usr/bin/env python3

import py_trees


def check_foo():
    blackboard = py_trees.blackboard.Client(name="Reader")
    blackboard.register_key(key="foo", access=py_trees.common.Access.READ)
    print("Foo: {}".format(blackboard.foo))


blackboard = py_trees.blackboard.Client(name="Writer")
blackboard.register_key(key="foo", access=py_trees.common.Access.WRITE)
blackboard.foo = "bar"
check_foo()
