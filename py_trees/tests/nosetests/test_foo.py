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
import rocon_uri
import rocon_console.console as console

##############################################################################
# Tests
##############################################################################

# def test_experiments():
#     print(console.bold + "\n****************************************************************************************" + console.reset)
#     print(console.bold + "* Experiments" + console.reset)
#     print(console.bold + "****************************************************************************************" + console.reset)
#     rocon_uri_string = 'rocon://'
#     rocon_uri_string = 'rocon:/turtlebot2/dude/hydro/precise#rocon_apps/chirp'
#     rocon_uri_object = rocon_uri.parse(rocon_uri_string)
#     print("Rocon URI Object: %s" % rocon_uri_object)  
#     rocon_uri_object2 = rocon_uri.parse('rocon:/turtlebot2|waiterbot/dude/hydro/precise#rocon_apps/chirp')
#     print("Rocon URI Object: %s" %  str(rocon_uri_object.hardware_platform))
#     print("Rocon URI Object : %s" %  rocon_uri_object.hardware_platform.string)
#     print("Rocon URI Object : %s" %  rocon_uri_object.hardware_platform.list)
#     print("Rocon URI Object2: %s" %  rocon_uri_object2.hardware_platform.string)
#     print("Rocon URI Object2: %s" %  rocon_uri_object2.hardware_platform.list)

# def test_invalid_elements():
#     print(console.bold + "\n****************************************************************************************" + console.reset)
#     print(console.bold + "* Raising on invalid elements" + console.reset)
#     print(console.bold + "****************************************************************************************" + console.reset)
# 
#     rocon_uri_string = 'rocon:/turtlebot2/dude/hydro/precise#rocon_apps/chirp'
#     # rocon_uri_string = 'rocon:/turtlebot2/dude/hydro/precise#rocon_apps/chirp'  # the empty hier-part also works
# 
#     invalid_schema = rocon_uri_string.replace('rocon', 'http')
#     print(console.cyan + " - %s" % invalid_schema + console.reset)
#     assert_raises(rocon_uri.RoconURIValueError, rocon_uri.parse, invalid_schema)
# 
#     invalid_hardware_platform = rocon_uri_string.replace('turtlebot2', 'foobar')
#     print(console.cyan + " - %s" % invalid_hardware_platform + console.reset)
#     assert_raises(rocon_uri.RoconURIValueError, rocon_uri.parse, invalid_hardware_platform)
# 
#     invalid_application_framework = rocon_uri_string.replace('hydro', 'dont_box_me_in')
#     print(console.cyan + " - %s" % invalid_application_framework + console.reset)
#     assert_raises(rocon_uri.RoconURIValueError, rocon_uri.parse, invalid_application_framework)
# 
#     invalid_operating_system = rocon_uri_string.replace('precise', 'bados')
#     print(console.cyan + " - %s" % invalid_operating_system + console.reset)
#     assert_raises(rocon_uri.RoconURIValueError, rocon_uri.parse, invalid_operating_system)
# 
# def test_stringify():
#     print(console.bold + "\n****************************************************************************************" + console.reset)
#     print(console.bold + "* String representation" + console.reset)
#     print(console.bold + "****************************************************************************************" + console.reset)
# 
#     rocon_uri_string = 'rocon:/turtlebot2/dude/hydro/precise#rocon_apps/chirp'
# 
#     print(console.cyan + " - %s" % rocon_uri_string + console.reset)
#     rocon_uri_object = rocon_uri.parse(rocon_uri_string)
#     assert str(rocon_uri_object) == rocon_uri_string
# def test_compatibility():
#     print(console.bold + "\n****************************************************************************************" + console.reset)
#     print(console.bold + "* Compatibility" + console.reset)
#     print(console.bold + "****************************************************************************************" + console.reset)
# 
#     rocon_uri_string = 'rocon:/turtlebot2/dude/hydro/precise'
#     print(console.cyan + " - %s  ~ %s" % (rocon_uri_string, rocon_uri_string) + console.reset)
#     assert(rocon_uri.is_compatible(rocon_uri_string, rocon_uri_string) == True)
#     # Missing operating system
#     modified_rocon_uri_string = 'rocon:/turtlebot2/dude/hydro'
#     print(console.cyan + " - %s  ~ %s" % (rocon_uri_string, modified_rocon_uri_string) + console.reset)
#     assert(rocon_uri.is_compatible(rocon_uri_string, modified_rocon_uri_string) == True)
#     # Missing application_framework/operating system
#     modified_rocon_uri_string = 'rocon:/turtlebot2/dude'
#     print(console.cyan + " - %s  ~ %s" % (rocon_uri_string, modified_rocon_uri_string) + console.reset)
#     assert(rocon_uri.is_compatible(rocon_uri_string, modified_rocon_uri_string) == True)
#     # Missing everything
#     modified_rocon_uri_string = 'rocon:/'
#     print(console.cyan + " - %s  ~ %s" % (rocon_uri_string, modified_rocon_uri_string) + console.reset)
#     assert(rocon_uri.is_compatible(rocon_uri_string, modified_rocon_uri_string) == True)
#     # Wildcards
#     modified_rocon_uri_string = 'rocon:/*/*/*/*'
#     print(console.cyan + " - %s  ~ %s" % (rocon_uri_string, modified_rocon_uri_string) + console.reset)
#     assert(rocon_uri.is_compatible(rocon_uri_string, modified_rocon_uri_string) == True)
#     # Regex names
#     modified_rocon_uri_string = 'rocon:/turtlebot2/dud*/hydro/precise'
#     print(console.cyan + " - %s  ~ %s" % (rocon_uri_string, modified_rocon_uri_string) + console.reset)
#     assert(rocon_uri.is_compatible(rocon_uri_string, modified_rocon_uri_string) == True)
#     modified_rocon_uri_string = 'rocon:/turtlebot2/dud*/hydro/precise'
#     print(console.cyan + " - %s  ~ %s" % (modified_rocon_uri_string, rocon_uri_string) + console.reset)
#     assert(rocon_uri.is_compatible(modified_rocon_uri_string, rocon_uri_string) == True)
#     doubly_modified_rocon_uri_string = 'rocon:/turtlebot2/dudette*/hydro/precise'
#     print(console.cyan + " - %s  ~ %s" % (modified_rocon_uri_string, doubly_modified_rocon_uri_string) + console.reset)
#     assert(rocon_uri.is_compatible(modified_rocon_uri_string, doubly_modified_rocon_uri_string) == True)
#     # No matching hardware platform
#     modified_rocon_uri_string = 'rocon:/pr2|waiterbot/dude'
#     print(console.cyan + " - %s !~ %s" % (rocon_uri_string, modified_rocon_uri_string) + console.reset)
#     assert(rocon_uri.is_compatible(rocon_uri_string, modified_rocon_uri_string) == False)
#     # Modified field
#     modified_rocon_uri_string = 'rocon:/turtlebot2/dudette/hydro/precise'
#     print(console.cyan + " - %s !~ %s" % (rocon_uri_string, modified_rocon_uri_string) + console.reset)
#     assert(rocon_uri.is_compatible(rocon_uri_string, modified_rocon_uri_string) == False)
# 
#     invalid_rocon_uri = 'rocon:/lala|turtlebot2'
#     try:
#         rocon_uri.is_compatible(rocon_uri_string, invalid_rocon_uri)
#     except rocon_uri.RoconURIValueError as e:
#         print(console.cyan + " - %s FAILS %s [%s]" % (rocon_uri_string, invalid_rocon_uri, str(e)) + console.reset)
#     assert_raises(rocon_uri.RoconURIValueError, rocon_uri.is_compatible, rocon_uri_string, invalid_rocon_uri)
