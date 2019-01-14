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
import re

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

def get_valid_filename(s: str) -> str:
    """
    Return the given string converted to a string that can be used for a clean
    filename (without extension). Remove leading and trailing spaces; convert
    other spaces and newlines to underscores; and remove anything that is not
    an alphanumeric, dash, underscore, or dot.

    .. code-block:: python

        >>> utilities.get_valid_filename("john's portrait in 2004.jpg")
        'johns_portrait_in_2004.jpg'

    Args:
        program (:obj:`str`): string to convert to a valid filename

    Returns:
        :obj:`str`: a representation of the specified string as a valid filename
    """
    s = str(s).strip().lower().replace(' ', '_').replace('\n', '_')
    return re.sub(r'(?u)[^-\w.]', '', s)

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
           print("Counter: {}".formta(foo.counter))
    """
    def decorate(func):
        for k in kwargs:
            setattr(func, k, kwargs[k])
        return func
    return decorate