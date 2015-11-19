/**
 * @file /gopher_configuration/src/lib/configuration.cpp
 */
/*****************************************************************************
** Includes
*****************************************************************************/

#include <ecl/console.hpp>
#include <ecl/exceptions.hpp>
#include <ros/ros.h>
#include "../../include/gopher_configuration/configuration.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace gopher_configuration {

/*****************************************************************************
** Implementation
*****************************************************************************/

Configuration::Configuration(const std::string& parameter_namespace)
: led_patterns()
{
  ros::NodeHandle nodehandle(parameter_namespace);
  bool result = true;
  if ( !nodehandle.getParam("actions", actions) ) {
    throw ecl::StandardException(LOC, ecl::NotFoundError,std::string("No configuration for actions under ") + parameter_namespace);
  }
  if (!nodehandle.getParam("battery", battery)) {
    throw ecl::StandardException(LOC, ecl::NotFoundError,std::string("No configuration for battery under ") + parameter_namespace);
  }
  if (!nodehandle.getParam("buttons", buttons)) {
    throw ecl::StandardException(LOC, ecl::NotFoundError,std::string("No configuration for buttons under ") + parameter_namespace);
  }
  if (!nodehandle.getParam("namespaces", namespaces)) {
    throw ecl::StandardException(LOC, ecl::NotFoundError,std::string("No configuration for namespaces under ") + parameter_namespace);
  }
  if (!nodehandle.getParam("services", services)) {
    throw ecl::StandardException(LOC, ecl::NotFoundError,std::string("No configuration for services under ") + parameter_namespace);
  }
  if (!nodehandle.getParam("sounds", sounds)) {
    throw ecl::StandardException(LOC, ecl::NotFoundError,std::string("No configuration for sounds under ") + parameter_namespace);
  }
  if (!nodehandle.getParam("topics", topics)) {
    throw ecl::StandardException(LOC, ecl::NotFoundError,std::string("No configuration for topics under ") + parameter_namespace);
  }
}

std::ostream& operator<<(std::ostream& stream, const Configuration& configuration) {
  stream << ecl::bold << ecl::white << "Gopher Configuration\n" << ecl::reset;
  stream << ecl::green << "  Actions:\n";
  for (const auto& key_value : configuration.actions) {
    stream << "    " << ecl::cyan << std::left << std::setw(20) << key_value.first << ecl::reset << ": " << ecl::yellow << key_value.second << "\n";
  }
  stream << ecl::green << "  Battery:\n";
  for (const auto& key_value : configuration.battery) {
    stream << "    " << ecl::cyan << std::left << std::setw(20) << key_value.first << ecl::reset << ": " << ecl::yellow << key_value.second << "\n";
  }
  stream << ecl::green << "  Buttons:\n";
  for (const auto& key_value : configuration.buttons) {
    stream << "    " << ecl::cyan << std::left << std::setw(20) << key_value.first << ecl::reset << ": " << ecl::yellow << key_value.second << "\n";
  }
  stream << configuration.led_patterns;
  stream << ecl::green << "  Namespaces:\n";
  for (const auto& key_value : configuration.namespaces) {
    stream << "    " << ecl::cyan << std::left << std::setw(20) << key_value.first << ecl::reset << ": " << ecl::yellow << key_value.second << "\n";
  }
  stream << ecl::green << "  Services:\n";
  for (const auto& key_value : configuration.services) {
    stream << "    " << ecl::cyan << std::left << std::setw(20) << key_value.first << ecl::reset << ": " << ecl::yellow << key_value.second << "\n";
  }
  stream << ecl::green << "  Sounds:\n";
  for (const auto& key_value : configuration.sounds) {
    stream << "    " << ecl::cyan << std::left << std::setw(20) << key_value.first << ecl::reset << ": " << ecl::yellow << key_value.second << "\n";
  }
  stream << ecl::green << "  Topics:\n";
  for (const auto& key_value : configuration.topics) {
    stream << "    " << ecl::cyan << std::left << std::setw(20) << key_value.first << ecl::reset << ": " << ecl::yellow << key_value.second << "\n";
  }
  stream << ecl::reset;
  stream.flush();
  return stream;
}

/*****************************************************************************
 ** Trailers
 *****************************************************************************/

} // namespace gopher_configuration
