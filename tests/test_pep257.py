# License: BSD
#   https://raw.githubusercontent.com/stonier/py_trees/devel/LICENSE
#

##############################################################################
# Imports
##############################################################################

from ament_pep257.main import main
import pytest

##############################################################################
# Tests
##############################################################################


@pytest.mark.linter
@pytest.mark.pep257
def test_pep257():
    # This only tests for flakes in the tests directory
    # TODO: enable for the src directory as well
    rc = main(argv=['.', 'test'])
    assert rc == 0, 'Found code style errors / warnings'
