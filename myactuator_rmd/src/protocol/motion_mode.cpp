/**
 * \file motion_mode.cpp
 * \mainpage
 *    V4.3 Motion Mode Control (MIT-style impedance controller)
 *    Bit-packed big-endian encoding/decoding for 8-byte CAN frames
*/

#include "myactuator_rmd/protocol/motion_mode.hpp"

#include <algorithm>
#include <cstdint>


namespace myactuator_rmd {

  constexpr std::uint16_t MotionMode::floatToUint(float val, float const min, float const max, std::uint16_t const bits) noexcept {
    float const span {max - min};
    val = std::clamp(val, min, max);
    return static_cast<std::uint16_t>((val - min) / span * static_cast<float>((1 << bits) - 1));
  }

  constexpr float MotionMode::uintToFloat(std::uint16_t const val, float const min, float const max, std::uint16_t const bits) noexcept {
    float const span {max - min};
    return static_cast<float>(val) / static_cast<float>((1 << bits) - 1) * span + min;
  }

  std::array<std::uint8_t, 8> MotionMode::encode(MotionModeCommand const& cmd, MotionModeLimits const& limits) noexcept {
    // Scale float parameters to integer ranges
    // p_des: 16-bit, v_des: 12-bit, kp: 12-bit, kd: 12-bit, t_ff: 12-bit
    std::uint16_t const p_int {floatToUint(cmd.position,  limits.position_min, limits.position_max, 16)};
    std::uint16_t const v_int {floatToUint(cmd.velocity,  limits.velocity_min, limits.velocity_max, 12)};
    std::uint16_t const kp_int {floatToUint(cmd.kp,       limits.kp_min,       limits.kp_max,       12)};
    std::uint16_t const kd_int {floatToUint(cmd.kd,       limits.kd_min,       limits.kd_max,       12)};
    std::uint16_t const t_int {floatToUint(cmd.torque_ff, limits.torque_min,   limits.torque_max,   12)};

    // Pack into 8 bytes big-endian:
    //   Byte 0: p_des[15:8]
    //   Byte 1: p_des[7:0]
    //   Byte 2: v_des[11:4]
    //   Byte 3: v_des[3:0] | kp[11:8]
    //   Byte 4: kp[7:0]
    //   Byte 5: kd[11:4]
    //   Byte 6: kd[3:0] | t_ff[11:8]
    //   Byte 7: t_ff[7:0]
    std::array<std::uint8_t, 8> data {};
    data[0] = static_cast<std::uint8_t>(p_int >> 8);
    data[1] = static_cast<std::uint8_t>(p_int & 0xFF);
    data[2] = static_cast<std::uint8_t>(v_int >> 4);
    data[3] = static_cast<std::uint8_t>(((v_int & 0x0F) << 4) | ((kp_int >> 8) & 0x0F));
    data[4] = static_cast<std::uint8_t>(kp_int & 0xFF);
    data[5] = static_cast<std::uint8_t>(kd_int >> 4);
    data[6] = static_cast<std::uint8_t>(((kd_int & 0x0F) << 4) | ((t_int >> 8) & 0x0F));
    data[7] = static_cast<std::uint8_t>(t_int & 0xFF);

    return data;
  }

  MotionModeFeedback MotionMode::decode(std::array<std::uint8_t, 8> const& data, MotionModeLimits const& limits) noexcept {
    // Unpack big-endian bit-packed fields:
    //   RX: motor_id[7:0] | p[15:8] | p[7:0] | v[11:4] | v[3:0]:t[11:8] | t[7:0] | ...
    // Per V4.3 protocol: DATA[0]=motor_id, DATA[1-2]=position(16), DATA[3-4]=velocity(12)+torque(12), DATA[5-7]=unused
    std::uint16_t const p_int {static_cast<std::uint16_t>((data[1] << 8) | data[2])};
    std::uint16_t const v_int {static_cast<std::uint16_t>((data[3] << 4) | (data[4] >> 4))};
    std::uint16_t const t_int {static_cast<std::uint16_t>(((data[4] & 0x0F) << 8) | data[5])};

    MotionModeFeedback feedback {};
    feedback.position = uintToFloat(p_int, limits.position_min, limits.position_max, 16);
    feedback.velocity = uintToFloat(v_int, limits.velocity_min, limits.velocity_max, 12);
    feedback.torque   = uintToFloat(t_int, limits.torque_min,   limits.torque_max,   12);

    return feedback;
  }

}
