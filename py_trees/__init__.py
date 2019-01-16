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

from . import behaviour  # noqa
from . import behaviours  # noqa
from . import blackboard  # noqa
from . import common  # noqa
from . import composites  # noqa
from . import console  # noqa
from . import decorators  # noqa
from . import demos  # noqa
from . import display  # noqa
from . import logging  # noqa
from . import meta  # noqa
from . import programs  # noqa
from . import syntax_highlighting  # noqa
from . import tests  # noqa
from . import timers  # noqa
from . import trees  # noqa
from . import utilities  # noqa
from . import visitors  # noqa

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
