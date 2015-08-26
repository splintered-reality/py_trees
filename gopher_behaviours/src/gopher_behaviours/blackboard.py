#!/usr/bin/env python

class Blackboard:
    """ Data store for the gopher delivery behaviours"""
    __shared_state = {
        'is_waiting': False,
        'traversed_locations': [],
        'remaining_locations': []
    }

    def __init__(self):
        self.__dict__ = self.__shared_state
