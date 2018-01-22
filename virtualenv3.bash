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
# Always pulling for now
pip install -r rtd-requirements.txt
python setup.py develop

echo ""
echo "Leave the virtual environment with 'deactivate'"
echo ""
echo "I'm grooty, you should be too."
echo ""

