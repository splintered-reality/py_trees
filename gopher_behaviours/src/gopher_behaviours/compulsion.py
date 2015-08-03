#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/yujinrobot/gopher_crazy_hospital/license/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
.. module:: compulsion
   :platform: Unix
   :synopsis: Mind control the little suckers.

Coerce the gophers into doing what you want them to do. Just a little bit
of planning and behavioural compulsion.

----

"""

##############################################################################
# Imports
##############################################################################

import logging
import py_trees

from . import battery
from . import delivery

##############################################################################
# Classes
##############################################################################


class Visitor:
    def __init__(self):
        self.logger = logging.getLogger("gopher_behaviours.Visitor")

    def run(self, behaviour):
        self.logger.debug("  %s [visited][%s]" % (behaviour.name, behaviour.status))


class PreTickVisitor:
    def __init__(self):
        self.logger = logging.getLogger("gopher_behaviours.PreTick")

    def run(self, behaviour_tree):
        self.logger.debug("\n--------- Run %s ---------\n" % behaviour_tree.count)


class GopherCompulsion(object):
    def __init__(self):
        battery_subtree = battery.create_battery_tree(name="Eating Disorder")
        quirky_deliveries = delivery.GopherDeliveries(name="Quirky Deliveries", locations=["beer_fridge", "pizza_shop", "sofa_in_front_of_tv", "anywhere_not_near_the_wife"])
        idle = py_trees.behaviours.Success("Idle")
        root = py_trees.Selector(name="Gopher Compulsion", children=[battery_subtree, quirky_deliveries.root, idle])
        self.tree = py_trees.BehaviourTree(root)

    def tick_tock(self):
        self.tree.visitors.append(Visitor())
        self.tree.tick_tock(sleep_ms=500, number_of_iterations=py_trees.CONTINUOUS_TICK_TOCK, pre_tick_visitor=self.pre_tick_visitor, post_tock_visitor=self.post_tock_visitor)

    def pre_tick_visitor(self, behaviour_tree):
        pass

    def post_tock_visitor(self, behaviour_tree):
        quirky_deliveries.post_tock_update()
