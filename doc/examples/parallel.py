#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import py_trees

if __name__ == '__main__':
    b1 = py_trees.behaviours.Success(name="B1")
    b2 = py_trees.behaviours.Success(name="B2")
    b3 = py_trees.behaviours.Success(name="B3")
    root = py_trees.composites.Parallel(
        policy=py_trees.common.ParallelPolicy.SuccessOnSelected(
            synchronise=True,
            children=[b1, b2]
        )
    )
    root.add_children([b1, b2, b3])
    py_trees.display.render_dot_tree(root, py_trees.common.string_to_visibility_level("all"))
