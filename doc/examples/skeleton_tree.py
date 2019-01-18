#!/usr/bin/env python

import py_trees

if __name__ == '__main__':

    root = py_trees.composites.Selector("Selector")
    high = py_trees.behaviours.Success(name="High Priority")
    med = py_trees.behaviours.Success(name="Med Priority")
    low = py_trees.behaviours.Success(name="Low Priority")
    root.add_children([high, med, low])

    behaviour_tree = py_trees.trees.BehaviourTree(root)
    behaviour_tree.setup(timeout=15)
    try:
        behaviour_tree.tick_tock(
            sleep_ms=500,
            number_of_iterations=py_trees.trees.CONTINUOUS_TICK_TOCK,
            pre_tick_handler=None,
            post_tick_handler=None
        )
    except KeyboardInterrupt:
        behaviour_tree.interrupt()
