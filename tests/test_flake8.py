# License: BSD
#   https://raw.githubusercontent.com/stonier/py_trees/devel/LICENSE
#

##############################################################################
# Imports
##############################################################################

import os
import sys

try:
    import pytest
    from ament_flake8.main import main
except ImportError as e:
    print("{}".format(str(e)))
    print("Skipping {}".format(__file__))
    sys.exit(1)

##############################################################################
# Logging
##############################################################################

# Disable alot of spam coming from pytest
from logging import getLogger
getLogger('flake8').propagate = False

##############################################################################
# Tests
##############################################################################


@pytest.mark.flake8
@pytest.mark.linter
def test_flake8():
    module_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'py_trees')
    test_path = os.path.dirname(__file__)
    config_file = os.path.join(os.path.dirname(__file__), 'setup.cfg')
    argv = [
        "--config",
        config_file,
        "paths",
        module_path,
        test_path
    ]
    # argv = ["--help"]
    rc = main(argv)
    assert rc == 0, 'Found errors'
