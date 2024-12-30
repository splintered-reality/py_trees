import importlib
import unittest


class ImportTest(unittest.TestCase):
    def test_import(self) -> None:
        """
        This test serves to make the buildfarm happy in Python 3.12 and later.
        See https://github.com/colcon/colcon-core/issues/678 for more information.
        """
        assert importlib.util.find_spec("py_trees")
