#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import py_trees

if __name__ == "__main__":
    worker = py_trees.behaviours.Success(name="Worker")
    on_completion = py_trees.behaviours.Success(name="On Completion")
    root = py_trees.idioms.eventually(
        name="Eventually",
        worker=worker,
        on_completion=on_completion,
    )
    py_trees.display.render_dot_tree(
        root, py_trees.common.string_to_visibility_level("all")
    )
