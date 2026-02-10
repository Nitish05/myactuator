/**
 * \file gains.hpp
 * \mainpage
 *    Contains struct for control loop gains (V4.3 float-based PID format)
 * \author
 *    Tobit Flatscher (github.com/2b-t)
*/

#ifndef MYACTUATOR_RMD__ACTUATOR_STATE__GAINS
#define MYACTUATOR_RMD__ACTUATOR_STATE__GAINS
#pragma once


namespace myactuator_rmd {

  /**\class PidGains
   * \brief
   *    Proportional, integral, and derivative gains for a PID controller (V4.3 format)
  */
  class PidGains {
    public:
      constexpr PidGains(float const kp_ = 0.0f, float const ki_ = 0.0f, float const kd_ = 0.0f) noexcept;
      PidGains(PidGains const&) = default;
      PidGains& operator = (PidGains const&) = default;
      PidGains(PidGains&&) = default;
      PidGains& operator = (PidGains&&) = default;

      float kp;
      float ki;
      float kd;
  };

  /**\class Gains
   * \brief
   *    Control loop gains for current, speed, and position loops
  */
  class Gains {
    public:
      constexpr Gains(PidGains const& current_ = {}, PidGains const& speed_ = {}, PidGains const& position_ = {}) noexcept;
      Gains(Gains const&) = default;
      Gains& operator = (Gains const&) = default;
      Gains(Gains&&) = default;
      Gains& operator = (Gains&&) = default;

      PidGains current;
      PidGains speed;
      PidGains position;
  };

  constexpr PidGains::PidGains(float const kp_, float const ki_, float const kd_) noexcept
  : kp{kp_}, ki{ki_}, kd{kd_} {
    return;
  }

  constexpr Gains::Gains(PidGains const& current_, PidGains const& speed_, PidGains const& position_) noexcept
  : current{current_}, speed{speed_}, position{position_} {
    return;
  }

}

#endif // MYACTUATOR_RMD__ACTUATOR_STATE__GAINS
