/**
 * @file /gopher_configuration/include/gopher_configuration/led_patterns.hpp
 */
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef gopher_configuration_LED_PATTERNS_HPP_
#define gopher_configuration_LED_PATTERNS_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ecl/console.hpp>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace gopher_configuration {

/*****************************************************************************
** Interfaces
*****************************************************************************/

class LEDPatterns {
public:
  LEDPatterns();

  // human interactions
  unsigned int humans_give_me_input;
  unsigned int humans_be_careful;
  unsigned int humans_i_need_help;
  unsigned int humans_fix_me_i_am_broken;
  // notifications
  unsigned int im_doing_something_cool;
  unsigned int holding;
  unsigned int dab_dab_hae;
  // states
  unsigned int error;

};

std::ostream& operator<<(std::ostream& stream, const LEDPatterns& patterns);

/*****************************************************************************
** Trailers
*****************************************************************************/

} // namespace gopher_configuration

#endif /* gopher_configuration_LED_PATTERNS_HPP_ */
