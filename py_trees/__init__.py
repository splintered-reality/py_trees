#
# License: BSD
#   https://raw.githubusercontent.com/stonier/py_trees/devel/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
This is the top-level namespace of the py_trees package.
"""
##############################################################################
# Imports
##############################################################################

from . import behaviour
from . import behaviours
from . import blackboard
from . import common
from . import composites
from . import console
from . import demos
from . import display
from . import logging
from . import meta
from . import programs
from . import syntax_highlighting
from . import tests
from . import timers
from . import trees
from . import utilities
from . import visitors

# really core conveniences (only the core ones please)
from .behaviour import Behaviour
from .blackboard import Blackboard
from .common import Status
from .composites import Composite, Selector, Sequence, Chooser
from .trees import BehaviourTree, CONTINUOUS_TICK_TOCK

##############################################################################
# Version
##############################################################################

from .version import __version__
