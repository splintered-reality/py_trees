#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import py_trees.decorators
import py_trees.display

def create_tasks():
    return [
        py_trees.behaviours.Count(
            name="R-R-S",
            fail_until=0,
            running_until=2,
            success_until=3,
            reset=True
        ),
        py_trees.behaviours.Count(
            name="R-S",
            fail_until=0,
            running_until=1,
            success_until=100,
            reset=True
        ),
    ]

if __name__ == '__main__':

    eternal_guard = py_trees.composites.Sequence(name="Eternal Guard", memory=False)
    conditions = py_trees.composites.Parallel(
        name="Conditions",
        policy=py_trees.common.ParallelPolicy.SuccessOnOne()
    )
    frssssf = py_trees.behaviours.Count(
        name="F-R-S-S-S-S-F",
        fail_until=1,
        running_until=2,
        success_until=6,
        reset=False
    )
    success = py_trees.behaviours.Success(name="Success")
    task_sequence = py_trees.composites.Sequence(name="Task Sequence", memory=True)
    tasks = create_tasks()

    eternal_guard.add_children([conditions, task_sequence])
    task_sequence.add_children(tasks)
    conditions.add_children([frssssf, success])
    py_trees.display.render_dot_tree(eternal_guard)
