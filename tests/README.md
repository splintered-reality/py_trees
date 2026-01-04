# Tests

Make sure you have an sourced the appropriate environment
(virtual or colcon) before executing.

# Executing Tests

```bash
# run all tests in the current directory
$ pytest

# All tests with full stdout (-s / --capture=no)
$ pytest -s

# A single test module
$ pytest -s test_alakazam.py

# A single test
$ pytest -s test_action_clients.py::test_success

# Using tox from the root dir
$ tox -l         # list runnable contexts
$ tox            # everything
$ tox -e py312   # tests only
$ tox -e check   # lint only
$ tox -e format  # format only
$ tox -e mypy312 # check types
```
