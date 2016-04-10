# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICTS
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""
Helper functions for bag files and timestamps.
"""

import time
import rospy


def stamp_to_str(t):
    """
    Convert a rospy.Time to a human-readable string.

    @param t: time to convert
    @type  t: rospy.Time
    """
    t_sec = t.to_sec()
    if t < rospy.Time.from_sec(60 * 60 * 24 * 365 * 5):
        # Display timestamps earlier than 1975 as seconds
        return '%.3fs' % t_sec
    else:
        return time.strftime('%b %d %Y %H:%M:%S', time.localtime(t_sec)) + '.%03d' % (t.nsecs / 1000000)

def get_start_stamp(topic):
    """
    Get the earliest timestamp in the topic.

    :param topic: topic tuple
    :type topic: ``DynamicTimeline.Topic`` named tuple
    :return: earliest timestamp, ``rospy.Time``
    """
    start_stamp = None
    try:
        start_stamp = topic.queue[0].stamp
    except IndexError:
        pass

    return start_stamp

def get_end_stamp(topic):
    """
    Get the latest timestamp in the topic.

    :param topic: topic tuple
    :type topic: ``DynamicTimeline.Topic`` named tuple
    :return: latest timestamp, ``rospy.Time``
    """
    end_stamp = None
    try:
        end_stamp = topic.queue[-1].stamp
    except IndexError:
        pass

    return end_stamp
