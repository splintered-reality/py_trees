#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import py_trees

if __name__ == '__main__':
    task_one = py_trees.behaviours.Count(
        name="Task 1",
        fail_until=0,
        running_until=2,
        success_until=10
    )
    task_two = py_trees.behaviours.Count(
        name="Task 2",
        fail_until=0,
        running_until=2,
        success_until=10
    )
    high_priority_interrupt = py_trees.decorators.RunningIsFailure(
        child=py_trees.behaviours.Periodic(
            name="High Priority",
            n=3
        )
    )
    piwylo = py_trees.idioms.pick_up_where_you_left_off(
        name="Tasks",
        tasks=[task_one, task_two]
    )
    root = py_trees.composites.Selector(name="Pick Up\nWhere You\nLeft Off")
    root.add_children([high_priority_interrupt, piwylo])
    py_trees.display.render_dot_tree(
        root,
        py_trees.common.string_to_visibility_level("all"))
