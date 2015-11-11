/**
 * @file /gopher_configuration/include/gopher_configuration/configuration.hpp
 */
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef gopher_configuration_CONFIGURATION_HPP_
#define gopher_configuration_CONFIGURATION_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <iostream>
#include <map>
#include <string>

#include "led_patterns.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace gopher_configuration {

/*****************************************************************************
** Interfaces
*****************************************************************************/

/**
 * @brief C++ interface to a gopher configuration.
 *
 * Contains all the (robot internal) configuration requirements for
 * robot gopher. Anything that is a convention that needs to be referenced
 * by multiple programs, we centralise here.
 *
 * Note that this does not contain any configuration for components external
 * to the robot, nor is it intended to.
 *
 * Unlike the python version of the same which has several ways to load
 * the interface, the c++ interface only posesses the means to load
 * from the rosparam server (keeping it simple for now).
 *
 * There is a convention to the way the parameters are expected to be found.
 *
 * - there are various groups under the /gopher/configuration namespace
 * - each group can be pulled as a key-value map from the rosparam server
 * - keys are always strings, values can be other, but not mixed
 *
 * If you want a gruop that has mixed values, you'll need to create a
 * specialised class and parse each value independantly (lots of work).
 *
 * @raise ecl::StandardException : with NotFoundError flag if certain parameters aren't found.
 */
class Configuration {
public:
  Configuration(const std::string& parameter_namespace = "/gopher/configuration");

  std::map<std::string, std::string> actions;
  std::map<std::string, int> battery;
  std::map<std::string, std::string> buttons;
  std::map<std::string, std::string> namespaces;
  std::map<std::string, std::string> services;
  std::map<std::string, std::string> sounds;
  std::map<std::string, std::string> topics;

  // custom groups
  LEDPatterns led_patterns;

private:
};

std::ostream& operator<<(std::ostream& stream, const Configuration& configuration);

/*****************************************************************************
** Trailers
*****************************************************************************/

} // namespace gopher_configuration

#endif /* gopher_configuration_CONFIGURATION_HPP_ */
