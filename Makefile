#############################################################################################
# Build Documentation
#############################################################################################

NAME=`./setup.py --name`
VERSION=`./setup.py --version`

help:
	@echo "Local Build"
	@echo "  build     : build the python package"
	@echo "  tests     : run all of the nosetests"
	@echo "  clean     : clean build/dist directories"
	@echo "Packages"
	@echo "  pypi      : upload the package to PyPI"
	@echo "  source_deb: source packaging (for ppas)"
	@echo "  deb       : build the deb"
	@echo "Documentation"
	@echo "  docs      : buidl sphinx documentation"

docs:
	PY_TREES_DISABLE_COLORS=1 sphinx-build -E -b html doc doc/html

build:
	python setup.py build

clean:
	-rm -f MANIFEST
	-rm -rf build dist
	-rm -rf deb_dist
	-rm -rf debian
	-rm -rf ../*.build
	-rm -rf *.tar.gz
	-rm -rf *.egg-info
	-rm -rf doc/html

source_package:
	python setup.py sdist

source_deb:
	rm -rf dist deb_dist
	python setup.py --command-packages=stdeb.command sdist_dsc

deb:
	rm -rf dist deb_dist
	python setup.py --command-packages=stdeb.command bdist_deb

# Pypi Packaging Tutorial: https://packaging.python.org/tutorials/packaging-projects/
#
# Use with a ~/.pypirc file:
#
# [distutils]
# index-servers =
#     pypi
#
# [pypi]
# repository = https://upload.pypi.org/legacy/
# username:<username>
# password:<password>
#
# Note, you probably need to register the first time.
# You can also send it to testpypi first if you wish (see tutorial).

pypi:
	python setup.py sdist bdist_wheel
	twine upload dist/*

pypi_test:
	python setup.py sdist bdist_wheel
	twine upload --repository-url https://test.pypi.org/legacy/ dist/*

tests:
	python setup.py test

.PHONY: tests clean
