#!/usr/bin/env python
#
# License: BSD
#   https://raw.githubusercontent.com/stonier/py_trees/devel/LICENSE
#
##############################################################################
# Imports
##############################################################################

# enable some python3 compatibility options:
# (unicode_literals not compatible with python2 uuid module)
from __future__ import absolute_import, print_function

import py_trees
import py_trees.console as console

##############################################################################
# Logging Level
##############################################################################

py_trees.logging.level = py_trees.logging.Level.DEBUG
logger = py_trees.logging.Logger("Nosetest")

##############################################################################
# Subtrees
##############################################################################

def test_oneshot_with_fail_causes_reentry():

    decorator_oneshot = lambda child : py_trees.decorators.OneShot(child=child)
    idiom_oneshot = lambda child : py_trees.idioms.oneshot(
        name="Oneshot",
        variable_name="oneshot",
        behaviour=child)
    
    for title, create_tree in [
            ("Idiom", idiom_oneshot),
            ("Decorator", decorator_oneshot)
        ]:
        
        # Setup
        console.banner("{} w/ Failure Causes Reentry".format(title))
        py_trees.blackboard.Blackboard.clear()

        # Tree
        fail_then_run = py_trees.behaviours.Count(
            name="Fail Then Run",
            fail_until=1,
            running_until=2,
            reset=False
        )
        oneshot = create_tree(fail_then_run)
                
        # Ticking
        py_trees.tests.tick_tree(
            tree=oneshot,
            from_tick=1,
            to_tick=1,
            print_snapshot=True,
            print_blackboard=True
        )
        print(console.green + "Asserts (After Failure)" + console.reset)
        print(console.cyan + "  OneShot Status: " +
              console.yellow + "{}".format(oneshot.status) + 
              console.reset + " [{}]".format(py_trees.common.Status.FAILURE)
              )
        print(console.cyan + "  Child Status  : " +
              console.yellow + "{}".format(fail_then_run.status) +
              console.reset + " [{}]".format(py_trees.common.Status.FAILURE)
              )
        assert(oneshot.status == py_trees.common.Status.FAILURE)
        assert(fail_then_run.status == py_trees.common.Status.FAILURE)

        # Ticking
        py_trees.tests.tick_tree(
            tree=oneshot,
            from_tick=2,
            to_tick=2,
            print_snapshot=True,
            print_blackboard=True
        )
        print(console.green + "Asserts (After Running)" + console.reset)
        print(console.cyan + "  OneShot Status: " +
              console.yellow + "{}".format(oneshot.status) + 
              console.reset + " [{}]".format(py_trees.common.Status.RUNNING)
              )
        print(console.cyan + "  Child Status  : " +
              console.yellow + "{}".format(fail_then_run.status) +
              console.reset + " [{}]".format(py_trees.common.Status.RUNNING)
              )
        assert(oneshot.status == py_trees.common.Status.RUNNING)
        assert(fail_then_run.status == py_trees.common.Status.RUNNING)
    
def test_oneshot_with_subtrees_and_interrupt():
    sequence_subtree = py_trees.composites.Sequence(
        name="Sequence",
        children=[
            py_trees.behaviours.Success(name="Success 1"),
            py_trees.behaviours.Success(name="Success 2")
        ]
    )
    selector_subtree = py_trees.composites.Selector(
        name="Selector",
        children=[
            py_trees.behaviours.Failure(name="Failure"),
            py_trees.behaviours.Success(name="Success")
        ]
    )
    for worker_subtree in [sequence_subtree, selector_subtree]:
        decorator_oneshot = py_trees.decorators.OneShot(child=worker_subtree)
        idiom_oneshot = py_trees.idioms.oneshot(
            name="OneShot",
            variable_name="oneshot",
            behaviour=worker_subtree
        )
        for title, oneshot in [("Idiom", idiom_oneshot), ("Decorator", decorator_oneshot)]:
             
            console.banner("{} w/ {} and Interrupt".format(title, worker_subtree.name))
            py_trees.blackboard.Blackboard.clear()
             
            # Tree with higher priority branch
            root = py_trees.composites.Selector(name="Root")
            fail_after_one = py_trees.behaviours.Count(
                name="HighPriority", fail_until=1, running_until=1, success_until=2)
            root.add_children([fail_after_one, oneshot])
             
            # Ticking
            py_trees.tests.tick_tree(
                tree=root,
                from_tick=1,
                to_tick=2,
                print_snapshot=True,
                print_blackboard=True
            )
            print(console.green + "Asserts (After Interrupt)" + console.reset)
            print(console.cyan + "  OneShot Status: " +
                  console.yellow + "{}".format(oneshot.status) + 
                  console.reset + " [{}]".format(py_trees.common.Status.INVALID)
                  )
            print(console.cyan + "  Child Status  : " +
                  console.yellow + "{}".format(worker_subtree.status) +
                  console.reset + " [{}]".format(py_trees.common.Status.INVALID)
                  )
            assert(oneshot.status == py_trees.common.Status.INVALID)
            assert(worker_subtree.status == py_trees.common.Status.INVALID)
            py_trees.tests.tick_tree(
                tree=root,
                from_tick=3,
                to_tick=3,
                print_snapshot=True,
                print_blackboard=True
            )
            print(console.green + "Asserts (After Re-entry)" + console.reset)
            print(console.cyan + "  OneShot Status: " +
                  console.yellow + "{}".format(oneshot.status) +
                  console.reset + " [{}]".format(py_trees.common.Status.SUCCESS)
                  )
            print(console.cyan + "  Child Status  : " +
                  console.yellow + "{}".format(worker_subtree.status) +
                  console.reset + " [{}]".format(py_trees.common.Status.INVALID)
                  )
            assert(oneshot.status == py_trees.common.Status.SUCCESS)
            assert(worker_subtree.status == py_trees.common.Status.INVALID)
