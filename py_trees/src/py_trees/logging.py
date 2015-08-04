#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/yujinrobot/gopher_crazy_hospital/license/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
.. module:: loggers
   :platform: Unix
   :synopsis: Logging facilities in py_trees.

Set the py_tree loggers and provide an override option.

----

"""

##############################################################################
# Imports
##############################################################################

from enum import IntEnum
import rocon_console.console as console

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
# Underlying Loggers
##############################################################################


def logdebug(msg):
    global level
    if level < Level.INFO:
        print(console.green + "[DEBUG] " + str(msg) + console.reset)


def loginfo(msg):
    global level
    if level < Level.WARN:
        print("[ INFO] " + str(msg))


def logwarn(msg):
    global level
    if level < Level.ERROR:
        print("console.yellow + [ WARN] " + str(msg) + console.reset)


def logerror(msg):
    print(console.red + "[ERROR] " + str(msg) + console.reset)


def set_loggers(info, warn, debug, error):
    """
      Override the py_tree logging functions.
      Note that the logging level will also be overridden (usually
      the external functions have their own trick to set this).
    """
    loginfo = info
    logwarn = warn
    logdebug = debug
    logerror = error


##############################################################################
# Logger Class
##############################################################################

class Logger(object):
    """
    :cvar override: whether or not the default python logger has been overridden.
    :vartype override: bool
    """

    def __init__(self, name=None):
        self.prefix = '{:<12}'.format(name) + " : " if name else ""

    def debug(self, msg):
        logdebug(self.prefix + msg)

    def info(self, msg):
        loginfo(self.prefix + msg)

    def warning(self, msg):
        logwarn(self.prefix + msg)

    def error(self, msg):
        logerror(self.prefix + msg)


def get_logger(name=None):
    return Logger(name)
