/**
 * \file can_baud_rate.hpp
 * \mainpage
 *    Communication Baud rate of the CAN bus
 * \author
 *    Tobit Flatscher (github.com/2b-t)
*/

#ifndef MYACTUATOR_RMD__ACTUATOR_STATE__CAN_BAUD_RATE
#define MYACTUATOR_RMD__ACTUATOR_STATE__CAN_BAUD_RATE
#pragma once

#include <cstdint>


namespace myactuator_rmd {

  /**\enum CanBaudRate
  * \brief
  *    Communication Baud rate of the CAN bus
  */
  enum class CanBaudRate: std::uint8_t {
    KBPS500 = 0,           // RS485@115200 + CAN@500K
    MBPS1 = 1,             // RS485@500K + CAN@1M
    RS485_1M_CAN_OFF = 2,  // RS485@1M, CAN disabled
    RS485_1_5M = 3,        // RS485@1.5M
    RS485_2_5M = 4         // RS485@2.5M
  };

}

#endif // MYACTUATOR_RMD__ACTUATOR_STATE__CAN_BAUD_RATE
