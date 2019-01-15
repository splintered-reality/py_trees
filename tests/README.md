# Tests

Make sure you source the virutal environment in all cases to run
tests in this way.

## Executing Tests

Always run tests from the root directory, since that is where
it will pick up the nosetests configuration in `setup.cfg`.

```
# All Tests via SetupTools (indirectly)
$ python setup.py nosetests
# All Tests via Nosetest (directly)
$ nosetests ./tests
# A single test
$ nosetests tests/test_oneshot.py
```