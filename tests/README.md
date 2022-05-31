# Tests

Make sure you have an sourced the appropriate environment
(virtual or colcon) before executing.

# Executing Tests

```bash
# run all tests in the current directory
$ pytest-3

# All tests with full stdout (-s / --capture=no)
$ pytest-3 -s

# A single test module
$ pytest-3 -s test_alakazam.py

# A single test
$ pytest-3 -s test_action_clients.py::test_success

# Using tox from the root dir
$ tox -l         # list runnable contexts
$ tox            # everything
$ tox -e py38    # tests only
$ tox -e flake8  # lint only
```
