#!/usr/bin/env python

import py_trees
from py_trees.blackboard import Blackboard


class LocationTraversalHandler(py_trees.Behaviour):

    def __init__(self, name):
        super(LocationTraversalHandler, self).__init__(name)
        self.blackboard = Blackboard()

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
