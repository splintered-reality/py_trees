#!/usr/bin/env python

import rospy
import py_trees
from py_trees.blackboard import Blackboard

class BlackboardHandlerBase(py_trees.Behaviour):

    def __init__(self, name):
        super(BlackboardHandlerBase, self).__init__(name)
        self.blackboard = Blackboard()

class LocationTraversalHandler(BlackboardHandlerBase):

    def __init__(self, name):
        super(LocationTraversalHandler, self).__init__(name)

    def update(self):
        if not hasattr(self.blackboard, 'traversed_locations'):
            self.feedback_message = 'Blackboard did not have attribute traversed_locations. Location traversal failed.'
            return py_trees.Status.FAILURE
        elif not hasattr(self.blackboard, 'remaining_locations'):
            self.feedback_message = 'Blackboard did not have attribute remaining_locations. Location traversal failed.'
            return py_trees.Status.FAILURE
        else:
            try:
                self.blackboard.traversed_locations.append(self.blackboard.remaining_locations.pop(0))
            except IndexError:
                self.feedback_message = 'Traversed location list was empty, but pop was attempted'
                return py_trees.Status.FAILURE
            self.feedback_message = 'Traversal sucessful. New location is {0}'.format(self.blackboard.traversed_locations[-1])
            return py_trees.Status.SUCCESS

class CheckBlackboardVariable(BlackboardHandlerBase):
    """Check the blackboard to see if it has a specific variable, and optionally
    whether that variable has a specific value

    """

    def __init__(self, name, var_name, check_expected=False, expected_value=None, invert=False):
        """:param name: name of the behaviour
        :param var_name: name of the variable to check
        :param check_expected: if true, check the given expected value. This
            will automatically be set to true if the expected value is not None
        :param expected_value: expected value of the variable
        :param invert: if true, check that the value of the variable is != expected_value, rather than ==
        """
        super(CheckBlackboardVariable, self).__init__(name)
        self.var_name = var_name
        self.expected_value = expected_value
        # if the user sets the expected value, we check it
        self.check_expected = check_expected or expected_value is not None
        self.invert = invert

    def update(self):
        # if the attribute doesn't exist, always fail
        if not hasattr(self.blackboard, self.var_name):
            self.feedback_message = 'blackboard variable {0} did not exist'.format(self.var_name)
            return py_trees.Status.FAILURE

        if self.check_expected:
            got_expected = getattr(self.blackboard, self.var_name) == self.expected_value
            got_expected = not got_expected if self.invert else got_expected

            if got_expected:
                self.feedback_message = 'blackboard variable {0} had expected value'.format(self.var_name)
                return py_trees.Status.SUCCESS
            else:
                self.feedback_message = 'blackboard variable {0} did not have expected value'.format(self.var_name)
                return py_trees.Status.FAILURE
        else:
            self.feedback_message = 'blackboard has variable {0}'.format(self.var_name)
            return py_trees.Status.SUCCESS
