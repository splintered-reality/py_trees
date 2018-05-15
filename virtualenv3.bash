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
# Pull all requirements - test/doc & install
pip install -r rtd-requirements.txt

# NB: This would only pull install_requires
python setup.py develop

echo ""
echo "Leave the virtual environment with 'deactivate'"
echo ""
echo "I'm grooty, you should be too."
echo ""

