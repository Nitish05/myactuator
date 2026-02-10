/**
 * \file gain_index.hpp
 * \mainpage
 *    Contains the V4.3 PID gain parameter indices
*/

#ifndef MYACTUATOR_RMD__ACTUATOR_STATE__GAIN_INDEX
#define MYACTUATOR_RMD__ACTUATOR_STATE__GAIN_INDEX
#pragma once

#include <cstdint>


namespace myactuator_rmd {

  /**\enum GainIndex
   * \brief
   *    V4.3 PID gain parameter index for commands 0x30/0x31/0x32
  */
  enum class GainIndex: std::uint8_t {
    CURRENT_KP = 0x01,
    CURRENT_KI = 0x02,
    CURRENT_KD = 0x03,
    SPEED_KP = 0x04,
    SPEED_KI = 0x05,
    SPEED_KD = 0x06,
    POSITION_KP = 0x07,
    POSITION_KI = 0x08,
    POSITION_KD = 0x09
  };

}

#endif // MYACTUATOR_RMD__ACTUATOR_STATE__GAIN_INDEX
