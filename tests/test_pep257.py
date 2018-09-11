# License: BSD
#   https://raw.githubusercontent.com/stonier/py_trees/devel/LICENSE
#

##############################################################################
# Imports
##############################################################################

try:
    from ament_pep257.main import main
    import pytest

    @pytest.mark.linter
    @pytest.mark.pep257
    def test_pep257():
        rc = main(argv=['.', 'test'])
        assert rc == 0, 'Found code style errors / warnings'
except:
    print("No ament in your environment, skipping {}".format(__file__))
