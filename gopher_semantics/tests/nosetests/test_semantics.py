#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/gopher_crazy_hospital/py_trees/license/LICENSE
#

##############################################################################
# Imports
##############################################################################

# enable some python3 compatibility options:
# (unicode_literals not compatible with python2 uuid module)
from __future__ import absolute_import, print_function

from nose.tools import assert_raises
import gopher_semantics
import os
import rocon_console.console as console

##############################################################################
# Globals
##############################################################################

yamlstations={"one": {'primary_id': 1, 'left_id': 2, 'right_id': 3, 'pose':{'x': 0, 'y': 0, 'theta': 0}},
              "two": {'primary_id': 4, 'left_id': 5, 'right_id': 6, 'pose':{'x': 0, 'y': 0, 'theta': 0}},
              "three": {'primary_id': 7, 'left_id': 8, 'right_id': 9, 'pose':{'x': 0, 'y': 0, 'theta': 0}},
              "four": {'primary_id': 3, 'left_id': 2, 'right_id': 1, 'pose':{'x': 0, 'y': 0, 'theta': 0}},
              "five": {'primary_id': 10, 'left_id': 1, 'right_id': 11, 'pose':{'x': 0, 'y': 0, 'theta': 0}},
              "six": {'primary_id': 4, 'left_id': 8, 'right_id': 1, 'pose':{'x': 0, 'y': 0, 'theta': 0}},
              "seven": {'primary_id': 7, 'left_id': 9, 'right_id': 8, 'pose':{'x': 0, 'y': 0, 'theta': 0}}
          }

stations = gopher_semantics.DockingStations(from_yaml_object=yamlstations)

##############################################################################
# Tests
##############################################################################

# match a station to the yaml
def matchstation(match, station):
    assert(station.primary_id == match['primary_id'])
    assert(station.left_id == match['left_id'])
    assert(station.right_id == match['right_id'])
    assert(station.pose.x == match['pose']['x'])
    assert(station.pose.y == match['pose']['y'])
    assert(station.pose.theta == match['pose']['theta'])

def matchstations(yamlnames, stations):
    for name in yamlnames:
        assert(name in stations)
        matchstation(yamlstations[name], stations[name])

def test_find_docking_stations():
    oned = stations.find_docking_stations_with_ar_marker_id(primary=1)
    assert(len(oned) == 1)
    matchstations(['one'], oned)
    
    twod = stations.find_docking_stations_with_ar_marker_id(left=5)
    assert(len(twod) == 1)
    matchstations(['two'], twod)

    threed = stations.find_docking_stations_with_ar_marker_id(right=9)
    assert(len(threed) == 1)
    matchstations(['three'], threed)

    fourd = stations.find_docking_stations_with_ar_marker_id(right=1, left=2)
    assert(len(fourd) == 1)
    matchstations(['four'], fourd)

    twosixd = stations.find_docking_stations_with_ar_marker_id(primary=4)
    assert(len(twosixd) == 2)
    matchstations(['two', 'six'], twosixd)

    onefourfivesixd = stations.find_docking_stations_with_ar_marker_id(id_list=[1])
    assert(len(onefourfivesixd) == 4)
    matchstations(['one', 'four', 'five', 'six'], onefourfivesixd)

    onefourd = stations.find_docking_stations_with_ar_marker_id(id_list=[1, 2])
    assert(len(onefourd) == 2)
    matchstations(['one', 'four'], onefourd)

    threesevend = stations.find_docking_stations_with_ar_marker_id(primary=7, id_list=[8,9])
    assert(len(threesevend) == 2)
    matchstations(['three', 'seven'], threesevend)

    zero = stations.find_docking_stations_with_ar_marker_id(primary=4, left=2, right=3)
    assert(len(zero) == 0)

if __name__ == '__main__':
    test_find_docking_stations()
