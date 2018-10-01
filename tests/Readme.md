# Tests

Make sure you source the virutal environment in all cases to run
tests in this way (if you are using a colcon environment, the
same commands will eventually hold once ros2's rosdep becomes
functional).

## Executing Tests (Project Level)

To run all the tests (concisely) from the root directory:

```
$ python3 setup.py test
# OR
$ nosetests ./tests
```

## Executing Tests (With Verbosity)

Configuration for nosetests to run with verbose detail is in `setup.cfg` in this folder.
Make sure you run nosetests from this folder to catch that configuration.

```
# All tests
nosetests
# Individual tests
nosetests test_oneshot.py
```
