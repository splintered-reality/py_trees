##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
project(gopher_configuration)

##############################################################################
# Find Packages
##############################################################################

find_package(catkin REQUIRED COMPONENTS roslint)

##############################################################################
# Project Configuration
##############################################################################

catkin_python_setup()

# Lint Python modules
file(GLOB_RECURSE ${PROJECT_NAME}_PY_SRCS
     RELATIVE ${PROJECT_SOURCE_DIR} src/*.py)
roslint_python(${${PROJECT_NAME}_PY_SRCS})

##############################################################################
# Catkin
##############################################################################

catkin_package()

##############################################################################
# Installs
##############################################################################

catkin_install_python(
    PROGRAMS
        scripts/gopher_configuration
    DESTINATION
        ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY param DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

