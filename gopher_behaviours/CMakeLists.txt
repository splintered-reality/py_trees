##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
project(gopher_behaviours)

##############################################################################
# Find Packages
##############################################################################

find_package(catkin REQUIRED dynamic_reconfigure rospy uuid_msgs unique_id)

##############################################################################
# Python Configuration
##############################################################################

catkin_python_setup()

##############################################################################
# Dynamic Reconfigure
##############################################################################

generate_dynamic_reconfigure_options(
    dynamic_reconfigure/CustomDeliveries.cfg
    dynamic_reconfigure/QuirkyDeliveries.cfg
)

##############################################################################
# Catkin
##############################################################################

catkin_package()
catkin_add_env_hooks(42.gopher_deliveries SHELLS sh DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)

##############################################################################
# Installs
##############################################################################

#install(FILES src/rocon_uri/rules/rules.yaml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rules.yaml)

catkin_install_python(
    PROGRAMS
        scripts/demo_docking
        scripts/demo_elf
        scripts/demo_finish_him
        scripts/demo_flash_and_wait
        scripts/demo_move_base
        scripts/demo_parking
        scripts/demo_homebase_teleport
        scripts/demo_simple_motions
        scripts/splintered_reality.py
        scripts/gopher_elevator_run
        scripts/gopher_delivery_run
        scripts/custom_delivery_overseer.py
    DESTINATION
        ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY dynamic_reconfigure DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

##############################################################################
# Unit Tests
##############################################################################

#if (CATKIN_ENABLE_TESTING)
#  add_subdirectory(tests)
#endif()

