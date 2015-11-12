/**
 * @file /gopher_configuration/src/lib/led_patterns.cpp
 */
/*****************************************************************************
** Includes
*****************************************************************************/

#include <gopher_std_msgs/Notification.h>
#include <map>
#include <string>
#include "../../include/gopher_configuration/led_patterns.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace gopher_configuration {

/*****************************************************************************
** Implementation
*****************************************************************************/

LEDPatterns::LEDPatterns()
: humans_give_me_input(gopher_std_msgs::Notification::FLASH_BLUE)
, humans_be_careful(gopher_std_msgs::Notification::FLASH_YELLOW)
, humans_i_need_help(gopher_std_msgs::Notification::FLASH_PURPLE)
, humans_fix_me_i_am_broken(gopher_std_msgs::Notification::FLASH_RED)
, im_doing_something_cool(gopher_std_msgs::Notification::AROUND_RIGHT_BLUE)
, holding(gopher_std_msgs::Notification::AROUND_RIGHT_GREEN)
, dab_dab_hae(gopher_std_msgs::Notification::AROUND_RIGHT_YELLOW)
, error(gopher_std_msgs::Notification::SOLID_RED)
{

}

std::ostream& operator<<(std::ostream& stream, const LEDPatterns& patterns) {
  std::map<std::string, std::string> pattern_strings{
    {"humans_give_me_input", "flashing blue"},
    {"humans_be_careful", "flashing yellow"},
    {"humans_i_need_help", "flashing purple"},
    {"humans_fix_me_i_am_broken", "flashing red"},
    {"im_doing_something_cool", "around blue"},
    {"holding", "around green"},
    {"dab_dab_hae", "around yellow"},
    {"error", "solid red"},
  };
  stream << ecl::green << "  LED Patterns:\n";
  for (const auto& key_value :  pattern_strings) {
    stream << "    " << ecl::cyan << std::left << std::setw(20) << key_value.first << ecl::reset << ": " << ecl::yellow << key_value.second << "\n";
  }
  return stream;
}

/*****************************************************************************
 ** Trailers
 *****************************************************************************/

} // namespace gopher_configuration
