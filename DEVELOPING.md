# Developing

[[Test-Format-Lint](#test-format-lint)] [[Documentation](#documentation)] [[Packaging]](#packaging)]


## Test-Format-Lint

Check against at least one of py38 / py310 [1].

```
# Auto-format your code (if using VSCode, install the ufmt extension)
$ poetry run tox -e format

# Style, Format
$ poetry run tox -e check

# Type-Check
$ poetry run mypy38

# Tests
$ poetry run tox -e py38
```

[1] CI will test against both python versions for you, but should you wish to do so locally, open up two VSCode windows, one with the project opened in the default [py38 devcontainer](.devcontainer) and the other with the [py310 devcontainer](.devcontainer/py310).

## Documentation

Generate the docs, view them from `./docs/html` in a browser.

```
# Install dependencies
$ poetry install --with docs

# Build
$ poetry run make -C docs html
```

On Doc dependency changes, export the requirements for ReadTheDocs

```
$ poetry export -f requirements.txt --with docs -o docs/requirements.txt
```

## Packaging

If you have permission to publish on pypi:

```
$ poetry config http-basic.pypi ${POETRY_HTTP_BASIC_PYPI_USERNAME} ${POETRY_HTTP_BASIC_PYPI_PASSWORD}
$ poetry publish
```
