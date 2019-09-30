#!/usr/bin/env python3

import py_trees


def check_foo():
    blackboard = py_trees.blackboard.Blackboard(name="Reader", read={"foo"})
    print("Foo: {}".format(blackboard.foo))


blackboard = py_trees.blackboard.Blackboard(name="Writer", write={"foo"})
blackboard.foo = "bar"
check_foo()
