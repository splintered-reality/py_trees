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

catkin_add_env_hooks(42.gopher_deliveries SHELLS sh DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)

##############################################################################
# Installs
##############################################################################

#install(FILES src/rocon_uri/rules/rules.yaml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rules.yaml)

catkin_install_python(
    PROGRAMS
        scripts/demo_flash_and_wait
        scripts/demo_homebase_recovery
        scripts/gopher_deliveries
        scripts/gopher_elevator_run
        scripts/gopher_delivery_run
        scripts/custom_delivery_overseer.py
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

