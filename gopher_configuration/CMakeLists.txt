##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
project(gopher_configuration)

##############################################################################
# Find Packages
##############################################################################

find_package(catkin
    REQUIRED
    COMPONENTS
        ecl_build
        ecl_console
        ecl_exceptions
        roscpp
        roslint
)

##############################################################################
# Exports
##############################################################################

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME} 
    CATKIN_DEPENDS
        ecl_build
        ecl_console
        ecl_exceptions
        roscpp
)

##############################################################################
# Project Configuration
##############################################################################

ecl_enable_cxx11_compiler()
catkin_python_setup()

# Lint Python modules
file(GLOB_RECURSE ${PROJECT_NAME}_PY_SRCS
     RELATIVE ${PROJECT_SOURCE_DIR} src/*.py)
roslint_python(${${PROJECT_NAME}_PY_SRCS})

# c++ implementation
include_directories(include ${catkin_INCLUDE_DIRS})
add_subdirectory(src)

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

