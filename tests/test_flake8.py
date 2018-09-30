# License: BSD
#   https://raw.githubusercontent.com/stonier/py_trees/devel/LICENSE
#

##############################################################################
# Imports
##############################################################################

import sys

try:
    import pytest
    from ament_flake8.main import main
except ImportError as e:
    print("{}".format(str(e)))
    print("Skipping {}".format(__file__))
    sys.exit(1)

##############################################################################
# Tests
##############################################################################


@pytest.mark.flake8
@pytest.mark.linter
def test_flake8():
    rc = main(argv=[])
    assert rc == 0, 'Found errors'
