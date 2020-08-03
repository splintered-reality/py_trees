#!/usr/bin/env python
#
# License: BSD
#   https://raw.githubusercontent.com/stonier/py_trees/devel/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
Assorted utility functions.
"""

##############################################################################
# Imports
##############################################################################

import os

##############################################################################
# System OS Tools
##############################################################################


def which(program):
    '''
    Wrapper around the command line 'which' program.

    Args:
        program (:obj:`str`): name of the program to find.

    Returns:
        :obj:`str`: path to the program or None if it doesnt exist.
    '''
    def is_exe(fpath):
        return os.path.isfile(fpath) and os.access(fpath, os.X_OK)

    fpath, unused_fname = os.path.split(program)
    if fpath:
        if is_exe(program):
            return program
    else:
        for path in os.environ["PATH"].split(os.pathsep):
            path = path.strip('"')
            exe_file = os.path.join(path, program)
            if is_exe(exe_file):
                return exe_file

    return None


##############################################################################
# Python Helpers
##############################################################################

def static_variables(**kwargs):
    """
    This is a decorator that can be used with python methods to attach 
    initialised static variables to the method.

    .. code-block:: python

       @static_variables(counter=0)
       def foo():
           foo.counter += 1
           print("Counter: {}".format(foo.counter))
    """
    def decorate(func):
        for k in kwargs:
            setattr(func, k, kwargs[k])
        return func
    return decorate
