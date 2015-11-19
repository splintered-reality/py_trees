/**
 * @file /gopher_configuration/applications/demo_node.cpp
 */
/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include "../../include/gopher_configuration/configuration.hpp"

/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char **argv) {

  ros::init(argc, argv, "gopher_configuration");
  gopher_configuration::Configuration configuration;
  std::cout << configuration << std::endl;
}
