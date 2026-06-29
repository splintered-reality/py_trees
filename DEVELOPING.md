# Developing

[[Setup](#setup)] [[Test-Format-Lint](#test-format-lint)] [[Documentation](#documentation)] [[Packaging](#packaging)]

This project uses

* [`uv`](https://docs.astral.sh/uv/) for environment and dependency management
* [`ruff`](https://docs.astral.sh/ruff/) for formatting & linting
* [`ty`](https://github.com/astral-sh/ty) for static type checking

## Setup

Install [uv](https://docs.astral.sh/uv/getting-started/installation/).
Then, sync the project.
This creates a `.venv` with all dev dependencies:

```
$ uv sync
```

That's it - prefix commands with `uv run` to execute them inside the environment,
or activate it with `source .venv/bin/activate`.

## Test-Format-Lint

```
# Auto-format your code (install the 'charliermarsh.ruff' extension for VSCode)
$ uv run ruff format

# Lint
$ uv run ruff check          # add --fix to auto-fix

# Type-Check
$ uv run ty check

# Tests
$ uv run pytest -s tests/
$ uv run pytest --cov=py_trees tests/   # with coverage
```

CI runs the tests against python 3.10, 3.12, and 3.14.
To test against a specific version locally, pass `--python`, e.g. `uv run --python 3.12 pytest -s tests/`.

## Documentation

Generate the docs, view them from `./docs/html` in a browser.

```
# Build
$ uv run make -C docs html
```

On doc dependency changes, export the requirements for ReadTheDocs:

```
$ uv export --no-hashes --no-emit-project --no-default-groups --group docs -o docs/requirements.txt
```

## Packaging

```
# Build the sdist & wheel into ./dist
$ uv build

# Publish to PyPI (requires credentials, e.g. UV_PUBLISH_TOKEN)
$ uv publish
```
