#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import py_trees

if __name__ == "__main__":
    worker = py_trees.behaviours.Success(name="Worker")
    on_failure = py_trees.behaviours.Success(name="On Failure")
    on_success = py_trees.behaviours.Success(name="On Success")
    root = py_trees.idioms.eventually_swiss(
        name="Eventually (Swiss)",
        workers=[worker],
        on_failure=on_failure,
        on_success=on_success,
    )
    py_trees.display.render_dot_tree(
        root, py_trees.common.string_to_visibility_level("all")
    )
