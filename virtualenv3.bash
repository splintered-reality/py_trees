#!/bin/bash

# Script for setting up the development environment.
#source /usr/share/virtualenvwrapper/virtualenvwrapper.sh

NAME=py3_trees
VERSION="--python=/usr/bin/python3"

if [ "${VIRTUAL_ENV}" == "" ]; then
  workon ${NAME}
  if [ $? -ne 0 ]; then
    mkvirtualenv ${VERSION} ${NAME}
  fi
fi

# Get all dependencies for testing, doc generation
pip install -e .[docs]
pip install -e .[test]

# NB: this automagically nabs install_requires
python setup.py develop

echo ""
echo "Leave the virtual environment with 'deactivate'"
echo ""
echo "I'm grooty, you should be too."
echo ""

