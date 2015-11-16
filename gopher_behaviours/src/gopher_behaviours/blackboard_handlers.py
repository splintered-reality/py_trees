#!/usr/bin/env python

import py_trees
from .blackboard import Blackboard

class BlackboardHandlerBase(py_trees.Behaviour):

    def __init__(self, name):
        super(BlackboardHandlerBase, self).__init__(name)
        self.blackboard = Blackboard()

class LocationTraversalHandler(BlackboardHandlerBase):

    def __init__(self, name):
        super(LocationTraversalHandler, self).__init__(name)
        
    def update(self):
        self.blackboard.traversed_locations.append(self.blackboard.remaining_locations.pop(0))
        return py_trees.Status.SUCCESS
        
class CheckBlackboardVariable(BlackboardHandlerBase):

    def __init__(self, name, var_name, expected_value=None):
        super(CheckBlackboardVariable, self).__init__(name)
        self.var_name = var_name
        self.expected_value = expected_value
        
    def update(self):
        if not hasattr(self.blackboard, self.var_name):
            self.feedback_message = "blackboard variable {0} did not exist".format(self.var_name)
            return py_trees.Status.FAILURE
        elif self.expected_value is not None and getattr(self.blackboard, self.var_name, None) == self.expected_value:
            self.feedback_message = "blackboard variable {0} had expected value".format(self.var_name)
            return py_trees.Status.SUCCESS
        else:
            self.feedback_message = "blackboard variable {0} did not have expected value".format(self.var_name)
            return py_trees.Status.FAILURE
        
