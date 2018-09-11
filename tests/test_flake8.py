# License: BSD
#   https://raw.githubusercontent.com/stonier/py_trees/devel/LICENSE
#

##############################################################################
# Imports
##############################################################################

try:
    from ament_flake8.main import main
    import pytest

    @pytest.mark.flake8
    @pytest.mark.linter
    def test_flake8():
        rc = main(argv=[])
        assert rc == 0, 'Found errors'
except:
    print("No ament in your environment, skipping {}".format(__file__))
