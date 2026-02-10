/**
 * \file motion_mode.hpp
 * \mainpage
 *    V4.3 Motion Mode Control (MIT-style impedance controller)
 *    Uses CAN ID 0x400+motor_id (TX) / 0x500+motor_id (RX)
 *    Bit-packed big-endian format for position, velocity, Kp, Kd, and torque
*/

#ifndef MYACTUATOR_RMD__PROTOCOL__MOTION_MODE
#define MYACTUATOR_RMD__PROTOCOL__MOTION_MODE
#pragma once

#include <array>
#include <cstdint>


namespace myactuator_rmd {

  /**\class MotionModeCommand
   * \brief
   *    MIT-style impedance controller command parameters
  */
  struct MotionModeCommand {
    float position {0.0f};     // desired position in radians
    float velocity {0.0f};     // desired velocity in rad/s
    float kp {0.0f};           // position gain
    float kd {0.0f};           // damping gain
    float torque_ff {0.0f};    // feed-forward torque in Nm
  };

  /**\class MotionModeFeedback
   * \brief
   *    MIT-style impedance controller feedback
  */
  struct MotionModeFeedback {
    float position {0.0f};     // actual position in radians
    float velocity {0.0f};     // actual velocity in rad/s
    float torque {0.0f};       // actual torque in Nm
  };

  /**\class MotionModeLimits
   * \brief
   *    Parameter range limits for scaling the 12/16 bit packed values
  */
  struct MotionModeLimits {
    float position_min {-12.5f};
    float position_max {12.5f};
    float velocity_min {-45.0f};
    float velocity_max {45.0f};
    float kp_min {0.0f};
    float kp_max {500.0f};
    float kd_min {0.0f};
    float kd_max {5.0f};
    float torque_min {-18.0f};
    float torque_max {18.0f};
  };

  /**\class MotionMode
   * \brief
   *    Static utility for encoding/decoding Motion Mode Control messages.
   *    Uses CAN ID 0x400+motor_id for TX and 0x500+motor_id for RX.
   *    Data is big-endian with bit-packed fields:
   *      TX: p_des[15:0] | v_des[11:0] | kp[11:0] | kd[11:0] | t_ff[11:0]
   *      RX: p[15:0] | v[11:0] | t[11:0]
  */
  class MotionMode {
    public:
      static constexpr std::uint32_t tx_offset {0x400};
      static constexpr std::uint32_t rx_offset {0x500};

      /**\fn encode
       * \brief
       *    Encode a MotionModeCommand into 8 bytes for CAN transmission
       *
       * \param[in] cmd
       *    The command parameters
       * \param[in] limits
       *    The parameter scaling limits
       * \return
       *    8-byte array ready for CAN transmission
      */
      [[nodiscard]]
      static std::array<std::uint8_t, 8> encode(MotionModeCommand const& cmd, MotionModeLimits const& limits = {}) noexcept;

      /**\fn decode
       * \brief
       *    Decode 8 bytes from CAN reception into MotionModeFeedback
       *
       * \param[in] data
       *    8-byte array from CAN reception
       * \param[in] limits
       *    The parameter scaling limits
       * \return
       *    Decoded feedback
      */
      [[nodiscard]]
      static MotionModeFeedback decode(std::array<std::uint8_t, 8> const& data, MotionModeLimits const& limits = {}) noexcept;

      /**\fn txCanId
       * \brief
       *    Get the TX CAN ID for a given motor ID
       *
       * \param[in] motor_id
       *    The motor ID [1, 32]
       * \return
       *    The CAN ID to send commands to
      */
      [[nodiscard]]
      static constexpr std::uint32_t txCanId(std::uint32_t const motor_id) noexcept {
        return tx_offset + motor_id;
      }

      /**\fn rxCanId
       * \brief
       *    Get the RX CAN ID for a given motor ID
       *
       * \param[in] motor_id
       *    The motor ID [1, 32]
       * \return
       *    The CAN ID to receive feedback from
      */
      [[nodiscard]]
      static constexpr std::uint32_t rxCanId(std::uint32_t const motor_id) noexcept {
        return rx_offset + motor_id;
      }

    private:
      [[nodiscard]]
      static constexpr std::uint16_t floatToUint(float val, float min, float max, std::uint16_t bits) noexcept;

      [[nodiscard]]
      static constexpr float uintToFloat(std::uint16_t val, float min, float max, std::uint16_t bits) noexcept;
  };

}

#endif // MYACTUATOR_RMD__PROTOCOL__MOTION_MODE
