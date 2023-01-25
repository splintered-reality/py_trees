#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import py_trees

if __name__ == '__main__':
    root = py_trees.composites.Sequence(name="Sequence with Memory", memory=True)
    guard = py_trees.behaviours.Success(name="Guard")
    a1 = py_trees.behaviours.Success(name="Action 1")
    a2 = py_trees.behaviours.Success(name="Action 2")
    a3 = py_trees.behaviours.Success(name="Action 3")
    root.add_children([guard, a1, a2, a3])
    py_trees.display.render_dot_tree(root, py_trees.common.string_to_visibility_level("all"))
