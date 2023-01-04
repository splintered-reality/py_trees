#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import py_trees

if __name__ == '__main__':

    eternal_guard = py_trees.composites.Sequence(name="Eternal Guard", memory=False)
    condition_one = py_trees.behaviours.StatusQueue(
        name="Condition 1",
        queue=[
            py_trees.common.Status.SUCCESS,
            py_trees.common.Status.FAILURE,
            py_trees.common.Status.SUCCESS,
        ],
        eventually=py_trees.common.Status.SUCCESS
    )
    condition_two = py_trees.behaviours.StatusQueue(
        name="Condition 2",
        queue=[
            py_trees.common.Status.SUCCESS,
            py_trees.common.Status.SUCCESS,
            py_trees.common.Status.FAILURE,
        ],
        eventually=py_trees.common.Status.SUCCESS
    )
    task_sequence = py_trees.composites.Sequence(name="Task Sequence", memory=True)
    task_one = py_trees.behaviours.Success(name="Worker 1")
    task_two = py_trees.behaviours.Running(name="Worker 2")

    eternal_guard.add_children([condition_one, condition_two, task_sequence])
    task_sequence.add_children([task_one, task_two])
    py_trees.display.render_dot_tree(eternal_guard)
