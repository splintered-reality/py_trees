#
# License: Yujin
#
##############################################################################
# Documentation
##############################################################################

"""
gopher_configuration standardises core configurable elements of a gopher
and convenient means for loading them onto and reading them (python modules
and c++ libraries) from the rosparam server.

It is extensible, in that it will also permit extra configurability beyond
the core configuration.

"""
##############################################################################
# Imports
##############################################################################

from .configuration import Configuration