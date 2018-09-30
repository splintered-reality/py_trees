# License: BSD
#   https://raw.githubusercontent.com/stonier/py_trees/devel/LICENSE
#

##############################################################################
# Imports
##############################################################################

import sys

try:
    import pytest
    # from ament_pep257.main import main
except ImportError as e:
    print("{}".format(str(e)))
    print("Skipping {}".format(__file__))
    sys.exit(1)

##############################################################################
# Tests
##############################################################################


@pytest.mark.linter
@pytest.mark.pep257
def test_pep257():
    print("")
    print("Choosing not to comply with PEP257 since it's incompatible")
    print("with what is used int his project: google style code strings.")
    print("")
    print("https://sphinxcontrib-napoleon.readthedocs.io/en/latest/example_google.html")
    print("")
    # rc = main(argv=['.', 'test'])
    # assert rc == 0, 'Found code style errors / warnings'
