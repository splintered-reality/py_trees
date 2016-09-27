#!/usr/bin/env python

import py_trees

if __name__ == '__main__':
    root = py_trees.composites.Sequence("Naive Context Switching")
    switch = py_trees.behaviours.Success(name="Switch Navi Context")
    move = py_trees.behaviours.Count(name="Move It", fail_until=0, running_until=3)
    restore = py_trees.behaviours.Success(name="Restore Navi Context")
    root.add_children([switch, move, restore])
    py_trees.display.render_dot_tree(root, py_trees.common.string_to_visibility_level("all"))

