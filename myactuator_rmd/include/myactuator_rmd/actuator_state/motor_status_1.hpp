/**
 * \file motor_status_1.hpp
 * \mainpage
 *    Contains struct with motor status
 * \author
 *    Tobit Flatscher (github.com/2b-t)
*/

#ifndef MYACTUATOR_RMD__ACTUATOR_STATE__MOTOR_STATUS_1
#define MYACTUATOR_RMD__ACTUATOR_STATE__MOTOR_STATUS_1
#pragma once

#include "myactuator_rmd/actuator_state/error_code.hpp"


namespace myactuator_rmd {

  /**\class MotorStatus1
   * \brief
   *    Motor status
  */
  class MotorStatus1 {
    public:
      constexpr MotorStatus1(int const temperature_ = 0, int const mos_temperature_ = 0,
                             bool const is_brake_released_ = false,
                             float const voltage_ = 0.0f, ErrorCode const error_code_ = ErrorCode::NO_ERROR) noexcept;
      MotorStatus1(MotorStatus1 const&) = default;
      MotorStatus1& operator = (MotorStatus1 const&) = default;
      MotorStatus1(MotorStatus1&&) = default;
      MotorStatus1& operator = (MotorStatus1&&) = default;

      int temperature;
      int mos_temperature;
      bool is_brake_released;
      float voltage;
      ErrorCode error_code;
  };

  constexpr MotorStatus1::MotorStatus1(int const temperature_, int const mos_temperature_,
                                       bool const is_brake_released_,
                                       float const voltage_, ErrorCode const error_code_) noexcept
  : temperature{temperature_}, mos_temperature{mos_temperature_},
    is_brake_released{is_brake_released_}, voltage{voltage_}, error_code{error_code_} {
    return;
  }

}

#endif // MYACTUATOR_RMD__ACTUATOR_STATE__MOTOR_STATUS_1
