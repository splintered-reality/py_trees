##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
project(py_trees)

##############################################################################
# Catkin
##############################################################################

find_package(catkin REQUIRED COMPONENTS py_trees_msgs)

catkin_python_setup()

catkin_package()


##############################################################################
# Installs
##############################################################################

#install(FILES src/rocon_uri/rules/rules.yaml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rules.yaml)

catkin_install_python(
    PROGRAMS
        scripts/demo_behaviour
        scripts/demo_dot_graphs
        scripts/demo_tree
        scripts/gopher_blackboard
    DESTINATION
        ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

##############################################################################
# Unit Tests
##############################################################################

if (CATKIN_ENABLE_TESTING)
  add_subdirectory(tests)
endif()

