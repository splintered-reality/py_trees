#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/stonier/py_trees_suite/license/LICENSE
#
##############################################################################
# Documentation
##############################################################################
"""
Test for CheckSubscriberVariable
"""
##############################################################################
# Imports
##############################################################################

from geometry_msgs.msg import Pose
import operator
import py_trees
# import rocon_console.console as console
import rospy
import unittest
import rostest



##############################################################################
# Tree
##############################################################################


class TestSubscriberNestedCheck(unittest.TestCase):
    '''
        Test for CheckSubscriberVariable
    '''
    def __init__(self, *args):
        super(TestSubscriberNestedCheck, self).__init__(*args)
        generate_failure = rospy.get_param("~failure", False)
        bad_topic = rospy.get_param("~bad_topic", False)
        topic_name = rospy.get_param("~topic_name", "pose_chatter") if not bad_topic else "wrong_topic"
        expected_value = 1.0 if not generate_failure else -1.0
        self.logger = py_trees.logging.get_logger("Subscriber Check")
        self.root = py_trees.Sequence(name="Demo Subscribers")
        check_subscriber_variable = py_trees.subscribers.CheckSubscriberVariable(
            "Check",
            topic_name=topic_name,
            topic_type=Pose,
            variable_name="position.x",
            expected_value=expected_value,
            comparison_operator=operator.eq
        )
        wait_for_subscriber = py_trees.subscribers.WaitForSubscriberData(
            "Wait",
            topic_name=topic_name,
            topic_type=Pose,
        )
        self.root.add_child(check_subscriber_variable)
        self.root.add_child(wait_for_subscriber)
        self.tree = py_trees.ROSBehaviourTree(self.root)
        rospy.on_shutdown(self.shutdown)
        # py_trees.display.render_dot_tree(self.root)
        py_trees.display.print_ascii_tree(self.root, 0)

    ##############################################################################
    # Tick Tock
    ##############################################################################

    def test_tick_tock(self):
        self.tree.visitors.append(py_trees.trees.DebugVisitor())
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.tree.tick(pre_tick_handler=self.pre_tick_handler)
            if self.root.status == py_trees.Status.SUCCESS or self.root.status == py_trees.Status.FAILURE:
                break
            rate.sleep()
        self.assertEquals(self.root.status, py_trees.Status.SUCCESS)

    def pre_tick_handler(self, behaviour_tree):
        self.logger.debug("")
        self.logger.debug("{:-^30}".format(" Run %s " % behaviour_tree.count))
        self.logger.debug("")

    def shutdown(self):
        self.tree.destroy()  # destroy the tree on shutdown to stop the behaviour
        self.tree.interrupt()

##############################################################################
# Main
##############################################################################

if __name__ == '__main__':
    py_trees.logging.level = py_trees.logging.Level.DEBUG
    rospy.init_node("test_subscriber_nested_check", log_level=rospy.DEBUG)
    rostest.rosrun('py_trees',
                   'test_subscriber_nested_check',
                   TestSubscriberNestedCheck)
