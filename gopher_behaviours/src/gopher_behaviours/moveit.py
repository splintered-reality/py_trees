#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/yujinrobot/gopher_crazy_hospital/license/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
.. module:: moveit
   :platform: Unix
   :synopsis: Moving behaviours

Kick the gophers around with these behaviours.

----

"""

##############################################################################
# Imports
##############################################################################

import logging
import py_trees

##############################################################################
# Class
##############################################################################


class Dock(py_trees.Behaviour):
    """
    Engage the docking behaviour
    """
    def __init__(self, name):
        super(Dock, self).__init__(name)

    def update(self):
        self.logger.debug("  %s [Dock::update()]" % self.name)
        return py_trees.Status.SUCCESS


class MoveToGoal(py_trees.Behaviour):
    def __init__(self, name, semantic_location="somewhere"):
        super(MoveToGoal, self).__init__(name)
        self.semantic_location = semantic_location

    def update(self):
        self.logger.debug("  %s [MoveToGoal::update()]" % self.name)
        return py_trees.Status.SUCCESS


class GoHome(py_trees.Behaviour):
    def __init__(self, name):
        super(GoHome, self).__init__(name)

    def update(self):
        self.logger.debug("  %s [GoHome::update()]" % self.name)
        return py_trees.Status.SUCCESS
