# All

```
python3 setup.py test
```

# Nosetests

Because I can never remember ... source the virtualenv/colcon environment
and execute in this folder:

```
# All tests
python3 -m "nose" --with-yanc
# Individual tests
python3 -m "nose" --with-yanc test_pep257.py
```

Right now, various options are in the setup.cfg (some problems with logging - see
notes in the .cfg file).

# Ament Tests

The ament tests are pytests - can also run them as follows (as long
as your ros environment is sourced):

```
# Test all under the current directory
py.test-3 tests

# Test a single test
py.test-3 tests/test_flakes.py
py.test-3 tests/test_pep257.py
```
# Issues

The olde `nosetests -s` does not work (though `nosetests -s <test_name>.py` does. This
also has the run on effect that `python setup.py test` is failing. I haven't
root caused this yet. Is this python3 or simply a later version on bionic (1.3.7)?
