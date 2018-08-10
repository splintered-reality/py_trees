# License: BSD
#   https://raw.githubusercontent.com/stonier/py_trees/devel/LICENSE
#

##############################################################################
# Imports
##############################################################################

from ament_flake8.main import main
import pytest

##############################################################################
# Tests
##############################################################################


@pytest.mark.flake8
@pytest.mark.linter
def test_flake8():
    # This only tests for flakes in the tests directory
    # TODO: enable for the src directory as well
    rc = main(argv=[])
    assert rc == 0, 'Found errors'
