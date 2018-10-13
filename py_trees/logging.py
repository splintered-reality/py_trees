#!/usr/bin/env python
#
# License: BSD
#   https://raw.githubusercontent.com/stonier/py_trees/devel/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
.. module:: loggers
   :synopsis: Logging facilities in py_trees.

Oh my spaghettified magnificence,
Bless my noggin with a tickle from your noodly appendages!
"""

##############################################################################
# Imports
##############################################################################

from enum import IntEnum

from . import console

##############################################################################
# Logging
##############################################################################

# I'd really prefer to use python logging facilities, but rospy logging
# on top of python logging kills it.
#
# Could still use it here, and would actually be useful if I could
# integrate it with nosetests, but for now, this will do.
# Note, you can get colour with python logging, but its tricky;
#
#   http://stackoverflow.com/questions/384076/how-can-i-color-python-logging-output
#
# python way:
#
#     import logging
#         logging.getLogger("py_trees.Behaviour")
#         logging.basicConfig(level=logging.DEBUG)
#
##############################################################################
# Level
##############################################################################


# levels
class Level(IntEnum):
    """
    An enumerator representing the logging level.
    Not valid if you override with your own loggers.
    """
    DEBUG = 0
    INFO = 1
    WARN = 2
    ERROR = 3


# module variable
level = Level.INFO

##############################################################################
# Logger Class
##############################################################################


class Logger(object):
    """
    :cvar override: whether or not the default python logger has been overridden.
    :vartype override: bool
    """

    def __init__(self, name=None):
        self.prefix = '{:<20}'.format(name.replace("\n", " ")) + " : " if name else ""

    def debug(self, msg):
        global level
        if level < Level.INFO:
            console.logdebug(self.prefix + msg)

    def info(self, msg):
        global level
        if level < Level.WARN:
            console.loginfo(self.prefix + msg)

    def warning(self, msg):
        global level
        if level < Level.ERROR:
            console.logwarn(self.prefix + msg)

    def error(self, msg):
        console.logerror(self.prefix + msg)
