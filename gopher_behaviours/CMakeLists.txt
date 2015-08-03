##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
project(gopher_behaviours)

##############################################################################
# Catkin
##############################################################################

find_package(catkin REQUIRED)
catkin_package()

catkin_python_setup()

##############################################################################
# Installs
##############################################################################

#install(FILES src/rocon_uri/rules/rules.yaml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rules.yaml)

#install(
#    PROGRAMS 
#        scripts/rocon_uri
#    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#    )

##############################################################################
# Unit Tests
##############################################################################

#if (CATKIN_ENABLE_TESTING)
#  add_subdirectory(tests)
#endif()

