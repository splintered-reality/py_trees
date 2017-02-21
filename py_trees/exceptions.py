#!/usr/bin/env python
#
# License: BSD
#   https://raw.githubusercontent.com/stonier/py_trees/devel/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
.. module:: exceptions
   :platform: Unix
   :synopsis: Various standard exceptions for py_tree behaviours and trees.

Oh my spaghettified magnificence,
Bless my noggin with a tickle from your noodly appendages!

----

"""

##############################################################################
# Imports
##############################################################################


class ConfigurationException(Exception):
    """
    For when a behaviour has not had its input arguments correctly configured.

    This should be thrown when the behaviour is in it's __init__ function.
    """
    pass


class NotSetupException(Exception):
    """
     For when a behaviour or tree has not had setup called on it, when it
     was required. Calling setup() is not a requirement of every behaviour
     or tree, but these execeptions are intended to make it transparent to
     the user when it should have been called.

     It should be thrown when the problem behaviour has had a first attempt
     at initialising.
    """
    pass


class SetupFailedException(Exception):
    """
     For when a behaviour or tree has had setup called on it, but failed and still
     proceeded to try and tick.
    """
    pass
