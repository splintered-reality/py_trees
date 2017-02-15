#!/usr/bin/env python
#
# License: BSD
#   https://raw.githubusercontent.com/stonier/py_trees_suite/devel/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
.. module:: utilities
   :synopsis: Assorted utility functions

Oh my spaghettified magnificence,
Bless my noggin with a tickle from your noodly appendages!
"""

##############################################################################
# Imports
##############################################################################

import os

##############################################################################
# Which
##############################################################################


def which(program):
    '''
      Wrapper around the command line 'which' program.

      :returns: path to the program or None if it doesnt exist.
      :rtype: str or None
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
