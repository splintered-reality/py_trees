##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
project(gopher_behaviours)

##############################################################################
# Catkin
##############################################################################

find_package(catkin REQUIRED rospy)
catkin_package()

catkin_python_setup()

##############################################################################
# Installs
##############################################################################

#install(FILES src/rocon_uri/rules/rules.yaml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rules.yaml)

catkin_install_python(
    PROGRAMS
        scripts/gopher_deliveries
        scripts/gopher_elevator_run
    DESTINATION
        ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
 
##############################################################################
# Unit Tests
##############################################################################

#if (CATKIN_ENABLE_TESTING)
#  add_subdirectory(tests)
#endif()

