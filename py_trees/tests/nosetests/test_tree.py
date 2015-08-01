#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/gopher_crazy_hospital/py_trees/license/LICENSE
#

##############################################################################
# Imports
##############################################################################

# enable some python3 compatibility options:
# (unicode_literals not compatible with python2 uuid module)
from __future__ import absolute_import, print_function

import logging
from nose.tools import assert_raises
import py_trees
import rocon_console.console as console

##############################################################################
# Logging Level
##############################################################################

logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger("py_trees.Test")

##############################################################################
# Classes
##############################################################################

class Count(py_trees.Behaviour):
    def __init__(self, name="Count", fail_until=3, running_until=5, success_until=6, *args, **kwargs):
        super(Count, self).__init__(name, *args, **kwargs)
        self.count = 0
        self.fail_until = fail_until
        self.running_until = running_until
        self.success_until = success_until
        self.number_count_resets = 0
        self.number_updated = 0
        self.logger = logging.getLogger("py_trees.Count")

    def terminate(self, new_status):
        self.logger.debug("  %s [terminate()]" % self.name)
        # reset only if udpate got us into an invalid state
        if new_status == py_trees.Status.INVALID:
            self.count = 0
            self.number_count_resets += 1

    def update(self):
        self.number_updated += 1
        self.count += 1
        if self.count <= self.fail_until:
            self.logger.debug("  %s [update()][%s -> FAILURE]" % (self.name, self.count))
            return py_trees.Status.FAILURE
        elif self.count <= self.running_until:
            self.logger.debug("  %s [update()][%s -> RUNNING]" % (self.name, self.count))
            return py_trees.Status.RUNNING
        elif self.count <= self.success_until:
            self.logger.debug("  %s [update()][%s -> SUCCESS]" % (self.name, self.count))
            return py_trees.Status.SUCCESS
        else:
            self.logger.debug("  %s [update()][%s -> FAILURE]" % (self.name, self.count))
            return py_trees.Status.FAILURE
    
    def __repr__(self):
        s =  "%s\n" % self.name
        s += "  Status : %s\n" % self.status
        s += "  Count  : %s\n" % self.count
        s += "  Resets : %s\n" % self.number_count_resets
        s += "  Updates: %s\n" % self.number_updated
        return s

class Visitor:
    def __init__(self):
        self.logger = logging.getLogger("py_trees.Visitor")

    def run(self, behaviour):
        self.logger.debug("  %s [visited][%s]" % (behaviour.name, behaviour.status))


def tick_tree(tree, visitor, from_iteration, to_iteration):
    print("\n================== Iteration %s-%s ==================\n" % (from_iteration, to_iteration))
    for i in range(from_iteration, to_iteration + 1):
        print("\n--------- Run %s ---------\n" % i)
        for node in tree.tick():
            node.visit(visitor)

def print_summary(nodes):
    print("\n--------- Summary ---------\n")
    for node in nodes:
        print("%s" % node)

##############################################################################
# Tests
##############################################################################

def test_selector_composite():
    print(console.bold + "\n****************************************************************************************" + console.reset)
    print(console.bold + "* Selector" + console.reset)
    print(console.bold + "****************************************************************************************" + console.reset)
    visitor = Visitor()
    tree = py_trees.Selector(name='Selector')
    a = Count(name="A")
    b = Count(name="B")
    c = Count(name="C", fail_until=0, running_until=3, success_until=15)
    tree.add_child(a)
    tree.add_child(b)
    tree.add_child(c)
    tick_tree(tree, visitor, 1, 3)
    print_summary(nodes=[a, b, c])
    print("--------- Assertions ---------\n")
    print("a.count == 3")
    assert(a.count == 3)
    print("a.status == py_trees.Status.FAILURE")
    assert(a.status == py_trees.Status.FAILURE)
    print("c.status == py_trees.Status.RUNNING")
    assert(c.status == py_trees.Status.RUNNING)
    print("c.number_count_resets == 0")
    assert(c.number_count_resets == 0)
    tick_tree(tree, visitor, 4, 4)
    print_summary(nodes=[a, b, c])
    print("--------- Assertions ---------\n")
    print("a.status == py_trees.Status.RUNNING")
    assert(a.status == py_trees.Status.RUNNING)
    print("c.number_count_resets == 1")
    assert(c.number_count_resets == 1)
    print("c.status == py_trees.Status.INVALID")
    assert(c.status == py_trees.Status.INVALID)
    tick_tree(tree, visitor, 5, 8)
    print_summary(nodes=[a, b, c])
    print("Done")

def test_sequence_composite():
    print(console.bold + "\n****************************************************************************************" + console.reset)
    print(console.bold + "* Sequence" + console.reset)
    print(console.bold + "****************************************************************************************" + console.reset)
    visitor = Visitor()
    tree = py_trees.Sequence(name='Sequence')
    a = Count(name="A", fail_until=0, running_until=3, success_until=6)
    b = Count(name="B", fail_until=0, running_until=3, success_until=6)
    c = Count(name="C", fail_until=0, running_until=3, success_until=6)
    tree.add_child(a)
    tree.add_child(b)
    tree.add_child(c)
    tick_tree(tree, visitor, 1, 5)
    print_summary(nodes=[a, b, c])
    print("--------- Assertions ---------\n")
    print("a.status == py_trees.Status.SUCCESS")
    assert(a.status == py_trees.Status.SUCCESS)
    print("b.status == py_trees.Status.RUNNING")
    assert(b.status == py_trees.Status.RUNNING)
    print("tree.status == py_trees.Status.RUNNING")
    assert(tree.status == py_trees.Status.RUNNING)
    tick_tree(tree, visitor, 6, 10)
    print_summary(nodes=[a, b, c])
    print("--------- Assertions ---------\n")
    print("a.status == py_trees.Status.SUCCESS")
    assert(a.status == py_trees.Status.SUCCESS)
    print("b.status == py_trees.Status.SUCCESS")
    assert(b.status == py_trees.Status.SUCCESS)
    print("b.status == py_trees.Status.SUCCESS")
    assert(c.status == py_trees.Status.SUCCESS)
    print("tree.status == py_trees.Status.SUCCESS")
    assert(tree.status == py_trees.Status.SUCCESS)
    tick_tree(tree, visitor, 11, 13)
    print_summary(nodes=[a, b, c])
    print("--------- Assertions ---------\n")
    print("a.status == py_trees.Status.FAILURE")
    assert(a.status == py_trees.Status.FAILURE)
    print("tree.status == py_trees.Status.FAILURE")
    assert(tree.status == py_trees.Status.FAILURE)
    
def test_mixed_tree():
    print(console.bold + "\n****************************************************************************************" + console.reset)
    print(console.bold + "* Mixed Tree" + console.reset)
    print(console.bold + "****************************************************************************************" + console.reset)
    visitor = Visitor()
    
    a = Count(name="A", fail_until=3, running_until=5, success_until=7)

    sequence = py_trees.Sequence(name="Sequence")
    b = Count(name="B", fail_until=0, running_until=3, success_until=5)
    c = Count(name="C", fail_until=0, running_until=3, success_until=5)
    sequence.add_child(b)
    sequence.add_child(c)

    d = Count(name="D", fail_until=0, running_until=3, success_until=15)

    root = py_trees.Selector(name="Root")
    print("Root.children: %s " % [child.name for child in root.children])
    root.add_child(a)
    root.add_child(sequence)
    root.add_child(d)

    tick_tree(root, visitor, 1, 2)
    print_summary(nodes=[a, b, c, d])
    print("--------- Assertions ---------\n")
    print("a.status == py_trees.Status.FAILURE")
    assert(a.status == py_trees.Status.FAILURE)
    print("sequence.status == py_trees.Status.RUNNING")
    assert(sequence.status == py_trees.Status.RUNNING)
    print("b.status == py_trees.Status.RUNNING")
    assert(b.status == py_trees.Status.RUNNING)
    print("root.status == py_trees.Status.RUNNING")
    assert(root.status == py_trees.Status.RUNNING)

    tick_tree(root, visitor, 3, 11)
    print_summary(nodes=[a, b, c, d])
    print("--------- Assertions ---------\n")
    print("a.status == py_trees.Status.FAILURE")
    assert(a.status == py_trees.Status.FAILURE)
    print("sequence.status == py_trees.Status.SUCCESS")
    assert(sequence.status == py_trees.Status.SUCCESS)
    print("c.status == py_trees.Status.SUCCESS")
    assert(c.status == py_trees.Status.SUCCESS)
    print("root.status == py_trees.Status.SUCCESS")
    assert(root.status == py_trees.Status.SUCCESS)

    tick_tree(root, visitor, 12, 13)
    print_summary(nodes=[a, b, c, d])
    print("--------- Assertions ---------\n")
    print("a.status == py_trees.Status.FAILURE")
    assert(a.status == py_trees.Status.FAILURE)
    print("sequence.status == py_trees.Status.FAILURE")
    assert(sequence.status == py_trees.Status.FAILURE)
    print("b.status == py_trees.Status.FAILURE")
    assert(b.status == py_trees.Status.FAILURE)
    print("d.status == py_trees.Status.RUNNING")
    assert(d.status == py_trees.Status.RUNNING)
    print("root.status == py_trees.Status.RUNNING")
    assert(root.status == py_trees.Status.RUNNING)


# def test_foo():
#     print('--------- Nosetest Logs ---------')
#     print_logging()
#     py_trees.foo1()
#     py_trees.foo2()
#     print('--------- Behaviour Logs ---------')
#     d = Count(name="D")
#     d.initialise()
#     print("Done")
#     assert(True)

##############################################################################
# Main
##############################################################################

# if __name__ == '__main__':
#     print_logging()
#     dude = py_trees.Behaviour(name="dude")
#    test_selector()

# def test_invalid_elements():
#     print(console.bold + "\n****************************************************************************************" + console.reset)
#     print(console.bold + "* Raising on invalid elements" + console.reset)
#     print(console.bold + "****************************************************************************************" + console.reset)
# 
#     rocon_uri_string = 'rocon:/turtlebot2/dude/hydro/precise#rocon_apps/chirp'
#     # rocon_uri_string = 'rocon:/turtlebot2/dude/hydro/precise#rocon_apps/chirp'  # the empty hier-part also works
# 
#     invalid_schema = rocon_uri_string.replace('rocon', 'http')
#     print(console.cyan + " - %s" % invalid_schema + console.reset)
#     assert_raises(rocon_uri.RoconURIValueError, rocon_uri.parse, invalid_schema)
# 
#     invalid_hardware_platform = rocon_uri_string.replace('turtlebot2', 'foobar')
#     print(console.cyan + " - %s" % invalid_hardware_platform + console.reset)
#     assert_raises(rocon_uri.RoconURIValueError, rocon_uri.parse, invalid_hardware_platform)
# 
#     invalid_application_framework = rocon_uri_string.replace('hydro', 'dont_box_me_in')
#     print(console.cyan + " - %s" % invalid_application_framework + console.reset)
#     assert_raises(rocon_uri.RoconURIValueError, rocon_uri.parse, invalid_application_framework)
# 
#     invalid_operating_system = rocon_uri_string.replace('precise', 'bados')
#     print(console.cyan + " - %s" % invalid_operating_system + console.reset)
#     assert_raises(rocon_uri.RoconURIValueError, rocon_uri.parse, invalid_operating_system)
# 
# def test_stringify():
#     print(console.bold + "\n****************************************************************************************" + console.reset)
#     print(console.bold + "* String representation" + console.reset)
#     print(console.bold + "****************************************************************************************" + console.reset)
# 
#     rocon_uri_string = 'rocon:/turtlebot2/dude/hydro/precise#rocon_apps/chirp'
# 
#     print(console.cyan + " - %s" % rocon_uri_string + console.reset)
#     rocon_uri_object = rocon_uri.parse(rocon_uri_string)
#     assert str(rocon_uri_object) == rocon_uri_string
# def test_compatibility():
#     print(console.bold + "\n****************************************************************************************" + console.reset)
#     print(console.bold + "* Compatibility" + console.reset)
#     print(console.bold + "****************************************************************************************" + console.reset)
# 
#     rocon_uri_string = 'rocon:/turtlebot2/dude/hydro/precise'
#     print(console.cyan + " - %s  ~ %s" % (rocon_uri_string, rocon_uri_string) + console.reset)
#     assert(rocon_uri.is_compatible(rocon_uri_string, rocon_uri_string) == True)
#     # Missing operating system
#     modified_rocon_uri_string = 'rocon:/turtlebot2/dude/hydro'
#     print(console.cyan + " - %s  ~ %s" % (rocon_uri_string, modified_rocon_uri_string) + console.reset)
#     assert(rocon_uri.is_compatible(rocon_uri_string, modified_rocon_uri_string) == True)
#     # Missing application_framework/operating system
#     modified_rocon_uri_string = 'rocon:/turtlebot2/dude'
#     print(console.cyan + " - %s  ~ %s" % (rocon_uri_string, modified_rocon_uri_string) + console.reset)
#     assert(rocon_uri.is_compatible(rocon_uri_string, modified_rocon_uri_string) == True)
#     # Missing everything
#     modified_rocon_uri_string = 'rocon:/'
#     print(console.cyan + " - %s  ~ %s" % (rocon_uri_string, modified_rocon_uri_string) + console.reset)
#     assert(rocon_uri.is_compatible(rocon_uri_string, modified_rocon_uri_string) == True)
#     # Wildcards
#     modified_rocon_uri_string = 'rocon:/*/*/*/*'
#     print(console.cyan + " - %s  ~ %s" % (rocon_uri_string, modified_rocon_uri_string) + console.reset)
#     assert(rocon_uri.is_compatible(rocon_uri_string, modified_rocon_uri_string) == True)
#     # Regex names
#     modified_rocon_uri_string = 'rocon:/turtlebot2/dud*/hydro/precise'
#     print(console.cyan + " - %s  ~ %s" % (rocon_uri_string, modified_rocon_uri_string) + console.reset)
#     assert(rocon_uri.is_compatible(rocon_uri_string, modified_rocon_uri_string) == True)
#     modified_rocon_uri_string = 'rocon:/turtlebot2/dud*/hydro/precise'
#     print(console.cyan + " - %s  ~ %s" % (modified_rocon_uri_string, rocon_uri_string) + console.reset)
#     assert(rocon_uri.is_compatible(modified_rocon_uri_string, rocon_uri_string) == True)
#     doubly_modified_rocon_uri_string = 'rocon:/turtlebot2/dudette*/hydro/precise'
#     print(console.cyan + " - %s  ~ %s" % (modified_rocon_uri_string, doubly_modified_rocon_uri_string) + console.reset)
#     assert(rocon_uri.is_compatible(modified_rocon_uri_string, doubly_modified_rocon_uri_string) == True)
#     # No matching hardware platform
#     modified_rocon_uri_string = 'rocon:/pr2|waiterbot/dude'
#     print(console.cyan + " - %s !~ %s" % (rocon_uri_string, modified_rocon_uri_string) + console.reset)
#     assert(rocon_uri.is_compatible(rocon_uri_string, modified_rocon_uri_string) == False)
#     # Modified field
#     modified_rocon_uri_string = 'rocon:/turtlebot2/dudette/hydro/precise'
#     print(console.cyan + " - %s !~ %s" % (rocon_uri_string, modified_rocon_uri_string) + console.reset)
#     assert(rocon_uri.is_compatible(rocon_uri_string, modified_rocon_uri_string) == False)
# 
#     invalid_rocon_uri = 'rocon:/lala|turtlebot2'
#     try:
#         rocon_uri.is_compatible(rocon_uri_string, invalid_rocon_uri)
#     except rocon_uri.RoconURIValueError as e:
#         print(console.cyan + " - %s FAILS %s [%s]" % (rocon_uri_string, invalid_rocon_uri, str(e)) + console.reset)
#     assert_raises(rocon_uri.RoconURIValueError, rocon_uri.is_compatible, rocon_uri_string, invalid_rocon_uri)
