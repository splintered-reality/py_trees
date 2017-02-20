##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
project(py_trees_ros)

##############################################################################
# Catkin
##############################################################################

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package()


##############################################################################
# Installs
##############################################################################

 catkin_install_python(
     PROGRAMS
         scripts/demo_tree
         scripts/blackboard_watcher
     DESTINATION
         ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

##############################################################################
# Unit Tests
##############################################################################

if (CATKIN_ENABLE_TESTING)
  #add_subdirectory(tests)
endif()
