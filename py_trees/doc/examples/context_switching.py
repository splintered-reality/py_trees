#!/usr/bin/env python

import py_trees


class NaviContext(py_trees.behaviour.Behaviour):
    def __init(self, name):
        super(NaviContext, self).__init__(name)

    def initialise(self):
        # save the original context and switch the navi context here
        pass

    def update(self):
        # just run, force life cycle changes via behaviour tree priority switches
        self.status = py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        # restore the navi context here
        pass


if __name__ == '__main__':
    root = py_trees.composites.Parallel("Custom Navigation", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
    context = NaviContext(name="Navi Context")
    move = py_trees.behaviours.Count(name="Move It", fail_until=0, running_until=3)
    root.add_children([context, move])
    py_trees.display.render_dot_tree(root, py_trees.common.string_to_visibility_level("all"))
