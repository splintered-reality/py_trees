#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import py_trees

if __name__ == '__main__':
    sequence = py_trees.composites.Sequence("Sequence")
    guard = py_trees.behaviours.Success(name="Guard")
    a1 = py_trees.behaviours.Success(name="Action 1")
    a2 = py_trees.behaviours.Success(name="Action 2")
    a3 = py_trees.behaviours.Success(name="Action 3")
    sequence.add_children([guard, a1, a2, a3])
    root = py_trees.idioms.oneshot(
        name="OneShot",
        variable_name="oneshot",
        behaviour=sequence,
        policy=py_trees.common.OneShotPolicy.ON_COMPLETION)
    py_trees.display.render_dot_tree(root, py_trees.common.string_to_visibility_level("all"))
