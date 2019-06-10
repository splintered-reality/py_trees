#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import py_trees

if __name__ == '__main__':
    root = py_trees.composites.Selector("Selector")
    high = py_trees.behaviours.Success(name="High Priority")
    med = py_trees.behaviours.Success(name="Med Priority")
    low = py_trees.behaviours.Success(name="Low Priority")
    root.add_children([high, med, low])
    py_trees.display.render_dot_tree(root, py_trees.common.string_to_visibility_level("all"))
