#!/usr/bin/env python
#
# License: BSD
#   https://raw.githubusercontent.com/splintered-reality/py_trees/devel/LICENSE
#

##############################################################################
# Imports
##############################################################################

import nose

import py_trees
import py_trees.console as console

from py_trees.blackboard import Blackboard, BlackboardClient, BlackboardClientFacade

##############################################################################
# Helpers
##############################################################################


class create_blackboard(object):

    def __enter__(self):
        self.blackboard = BlackboardClient(name="bb")
        self.blackboard.register_key("foo", write=True)
        self.blackboard.foo = "bar"
        return self.blackboard

    def __exit__(self, unused_type, unused_value, unused_traceback):
        self.blackboard.unregister(clear=True)


##############################################################################
# Tests
##############################################################################


def test_standalone_blackboard_and_facade():
    console.banner("Standalone Client & Facade")
    with create_blackboard() as (blackboard):
        print('{0}'.format(blackboard))
        facade = BlackboardClientFacade(prefix="dude_", blackboard=blackboard)
        facade.register_key(key="bob", write=True)
        facade.register_key(key="jane", read=True)
        facade.bob = "cool bloke"
        assert("cool bloke" == facade.bob)
        assert(facade.exists("bob"))
        assert(blackboard.exists("dude_bob"))
        print('{0}'.format(blackboard))
        # could test lots more


def test_behaviour_and_facade():
    console.banner("Behaviour & Facade")

    class Foo(py_trees.behaviour.Behaviour):
        def __init__(self):
            super().__init__(name="Foo")
            self.blackboard.register_key("foo", write=True)
            self.blackboard.foo = "bar"
            self.agent_blackboard = BlackboardClientFacade(
                prefix="agent0_",
                blackboard=self.blackboard
            )
            self.agent_blackboard.register_key(key="initial_position", write=True)
            self.agent_blackboard.initial_position = 5.0

    foo = Foo()
    assert(Blackboard.get("foo") == "bar")
    assert(Blackboard.get("agent0_initial_position") == 5.0)
    print("{}".format(foo.blackboard))
