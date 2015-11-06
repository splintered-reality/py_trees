#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2009, Willow Garage, Inc.
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
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# import unittest
# import rospkg

# import tf
# from tf.srv import *

# # get mock from pypi as 'mock'
# from mock import Mock, MagicMock, patch

# from rqt_tf_tree.dotcode_tf import RosTfTreeDotcodeGenerator


# class DotcodeGeneratorTest(unittest.TestCase):

#     def test_generate_dotcode(self):
#         with patch('tf.TransformListener') as tf:
#             def tf_srv_fun_mock():
#                 return tf

#             yaml_data = {'frame1': {'parent': 'fr_parent',
#                                             'broadcaster': 'fr_broadcaster',
#                                             'rate': 'fr_rate',
#                                             'buffer_length': 'fr_buffer_length',
#                                             'most_recent_transform': 'fr_most_recent_transform',
#                                             'oldest_transform': 'fr_oldest_transform',}}
#             tf.frame_yaml = str(yaml_data)
            
#             factoryMock = Mock()
#             graphMock = Mock()
#             timeMock = Mock()
#             timerMock = Mock()
#             timerMock.now.return_value=timeMock
#             timeMock.to_sec.return_value=42

#             yamlmock = Mock()
#             yamlmock.load.return_value = yaml_data
            
#             factoryMock.create_dot.return_value = "foo"
#             factoryMock.get_graph.return_value = graphMock
            
#             gen = RosTfTreeDotcodeGenerator(0)
#             graph = gen.generate_dotcode(factoryMock, tf_srv_fun_mock, timerMock)

#             timerMock.now.assert_called_with()
#             timeMock.to_sec.assert_called_with()
#             factoryMock.create_dot.assert_called_with(graphMock)
            
#             self.assertEqual(graph, 'foo')
