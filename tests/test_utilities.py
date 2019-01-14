#!/usr/bin/env python
#
# License: BSD
#   https://raw.githubusercontent.com/stonier/py_trees/devel/LICENSE
#

##############################################################################
# Imports
##############################################################################

import py_trees
import py_trees.console as console
import py_trees.utilities as utilities

##############################################################################
# Tests
##############################################################################


def test_valid_filenames():
    console.banner("Valid Filenames")
    names = {
        "With\nNewlines": "with_newlines",
        "With Spaces": "with_spaces",
        " Leading Space": "leading_space",
        "Trailing Space ": "trailing_space"
    }
    for name, expected_name in names.items():
        print(console.cyan + repr(name) + ": " + console.yellow + expected_name + " [" + utilities.get_valid_filename(name) + "]" + console.reset)
        assert(utilities.get_valid_filename(name) == expected_name)