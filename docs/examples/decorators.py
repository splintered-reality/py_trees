#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Example demonstrating the use of some simple decorator nodes."""

import py_trees.decorators
import py_trees.display

if __name__ == "__main__":
    root = py_trees.composites.Sequence(name="Life", memory=False)
    timeout = py_trees.decorators.Timeout(
        name="Timeout", child=py_trees.behaviours.Success(name="Have a Beer!")
    )
    failure_is_success = py_trees.decorators.Inverter(
        name="Inverter", child=py_trees.behaviours.Success(name="Busy?")
    )
    root.add_children([failure_is_success, timeout])
    py_trees.display.render_dot_tree(root)
