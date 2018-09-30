# All

```
python3 setup.py test
```

# Nosetests

Because I can never remember...execute in this folder:

```
# All tests
python3 -m "nose" --with-yanc
# Individual tests
python3 -m "nose" --with-yanc test_pep257.py
```

Right now, various options are in the setup.cfg (some problems with logging - see
notes in the .cfg file).

# Pytests

The colcon tests are pytests - can also run them as follows:

```
# Test all under the current directory
py.test-3 tests

# Test a single test
py.test-3 tests/test_flakes.py
py.test-3 tests/test_pep257.py
```
