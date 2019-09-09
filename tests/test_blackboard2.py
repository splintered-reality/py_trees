#!/usr/bin/env python
#
# License: BSD
#   https://raw.githubusercontent.com/splintered-reality/py_trees/devel/LICENSE
#

##############################################################################
# Imports
##############################################################################

import uuid

import py_trees
import py_trees.console as console

from py_trees.blackboard2 import Blackboard

##############################################################################
# Helpers
##############################################################################


class create_blackboards(object):

    def __enter__(self):
        self.foo = create_blackboard_foo()
        self.bar = create_blackboard_bar()
        return (self.foo, self.bar)

    def __exit__(self, unused_type, unused_value, unused_traceback):
        self.foo.unregister()
        self.bar.unregister()


def create_blackboard_foo():
    """
    Create a blackboard client with a few variables.
    """
    blackboard = Blackboard(
        name="foo",
        unique_identifier=uuid.uuid4(),
        read=['foo', 'bar'],
        write=['foo', 'dude'],
    )
    # blackboard.foo = "more bar"  # leave one uninitialised
    blackboard.dude = "bob"
    return blackboard


def create_blackboard_bar():
    """
    Create another blackboard client with a few variables.
    """
    blackboard = Blackboard(
        name="foo",
        unique_identifier=uuid.uuid4(),
        read=['dude'],
        write=['bar', 'dudette'],
    )
    # blackboard.foo = "more bar"  # leave one uninitialised
    blackboard.bar = "less bar"
    blackboard.dudette = "Anna"
    return blackboard

##############################################################################
# Tests
##############################################################################


def test_client_print_blackboard():
    console.banner("Client Construction & Print")
    with create_blackboards() as (foo, bar):
        print('{0}'.format(foo))
        print('{0}'.format(bar))
    assert(True)


def test_blackboard_key_accessors():
    console.banner("Key Accessors")
    with create_blackboards() as (foo, bar):
        no_of_keys = len(Blackboard.keys())
        print("# Registered keys: {} [{}]".format(no_of_keys, 4))
        assert(no_of_keys == 4)
        no_of_keys = len(Blackboard.keys_filtered_by_regex("dud"))
        print("# Keys by regex 'dud': {} [{}]".format(no_of_keys, 2))
        assert(no_of_keys == 2)
        no_of_keys = len(Blackboard.keys_filtered_by_clients({foo.unique_identifier}))
        print("# Keys by id [foo.id] {} [{}]".format(no_of_keys, 3))
        assert(no_of_keys == 3)
        no_of_keys = len(Blackboard.keys_filtered_by_clients({bar.unique_identifier}))
        print("# Keys by id [bar.id] {} [{}]".format(no_of_keys, 3))
        assert(no_of_keys == 3)
        no_of_keys = len(Blackboard.keys_filtered_by_clients({foo.unique_identifier, bar.unique_identifier}))
        print("# Keys by id [foo.id, bar.id] {} [{}]".format(no_of_keys, 4))
        assert(no_of_keys == 4)
        # show the convenience list -> set helper is ok
        no_of_keys = len(Blackboard.keys_filtered_by_clients([foo.unique_identifier]))
        print("# Can pass in a list instead of a set: True")
        assert(no_of_keys == 3)
