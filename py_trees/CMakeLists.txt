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
        scripts/py-trees-demo-behaviour-lifecycle
        scripts/py-trees-demo-tree
        scripts/py-trees-demo-dot-graphs
    DESTINATION
        ${CATKIN_PACKAGE_BIN_DESTINATION}
)

##############################################################################
# Unit Tests
##############################################################################

if (CATKIN_ENABLE_TESTING)
  add_subdirectory(tests)
endif()
