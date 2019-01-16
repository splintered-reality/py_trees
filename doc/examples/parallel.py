#!/usr/bin/env python

import py_trees

if __name__ == '__main__':
    root = py_trees.composites.Parallel("Parallel")
    b1 = py_trees.behaviours.Success(name="B1")
    b2 = py_trees.behaviours.Success(name="B2")
    root.add_children([b1, b2])
    py_trees.display.render_dot_tree(root, py_trees.common.string_to_visibility_level("all"))
