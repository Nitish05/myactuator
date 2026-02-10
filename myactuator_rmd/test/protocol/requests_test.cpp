/**
 * \file requests.cpp
 * \mainpage
 *    Tests for parsing of different request messages
 * \author
 *    Tobit Flatscher (github.com/2b-t)
*/

#include <chrono>
#include <cstdint>

#include <gtest/gtest.h>

#include "myactuator_rmd/actuator_state/acceleration_type.hpp"
#include "myactuator_rmd/actuator_state/can_baud_rate.hpp"
#include "myactuator_rmd/actuator_state/gain_index.hpp"
#include "myactuator_rmd/actuator_state/gains.hpp"
#include "myactuator_rmd/protocol/requests.hpp"


namespace myactuator_rmd {
  namespace test {

    TEST(GetCanIdRequestTest, parsing) {
      myactuator_rmd::GetCanIdRequest const request {{0x79, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00}};
      bool const is_write {request.isWrite()};
      EXPECT_EQ(is_write, false);
    }

    TEST(SetCanBaudRate0RequestTest, parsing) {
      myactuator_rmd::SetCanBaudRateRequest const request {{0xB4, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00}};
      CanBaudRate const baud_rate {request.getBaudRate()};
      EXPECT_EQ(baud_rate, CanBaudRate::KBPS500);
    }

    TEST(SetCanBaudRate1RequestTest, parsing) {
      myactuator_rmd::SetCanBaudRateRequest const request {{0xB4, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01}};
      CanBaudRate const baud_rate {request.getBaudRate()};
      EXPECT_EQ(baud_rate, CanBaudRate::MBPS1);
    }

    TEST(SetEncoderZeroRequestTest, parsing) {
      myactuator_rmd::SetEncoderZeroRequest const response {{0x63, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00}};
      auto const encoder_zero {response.getEncoderZero()};
      EXPECT_EQ(encoder_zero, 10000);
    }

    TEST(SetPositionPlanningAccelerationRequestTest, parsing) {
      myactuator_rmd::SetAccelerationRequest const request {{0x43, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00}};
      std::uint32_t const acceleration {request.getAcceleration()};
      AccelerationType const mode {request.getMode()};
      EXPECT_EQ(acceleration, 10000);
      EXPECT_EQ(mode, AccelerationType::POSITION_PLANNING_ACCELERATION);
    }

    TEST(SetPositionPlanningDecelerationRequestTest, parsing) {
      myactuator_rmd::SetAccelerationRequest const request {{0x43, 0x01, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00}};
      std::uint32_t const acceleration {request.getAcceleration()};
      AccelerationType const mode {request.getMode()};
      EXPECT_EQ(acceleration, 10000);
      EXPECT_EQ(mode, AccelerationType::POSITION_PLANNING_DECELERATION);
    }

    TEST(SetVelocityPlanningAccelerationRequestTest, parsing) {
      myactuator_rmd::SetAccelerationRequest const request {{0x43, 0x02, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00}};
      std::uint32_t const acceleration {request.getAcceleration()};
      AccelerationType const mode {request.getMode()};
      EXPECT_EQ(acceleration, 10000);
      EXPECT_EQ(mode, AccelerationType::VELOCITY_PLANNING_ACCELERATION);
    }

    TEST(SetVelocityPlanningDecelerationRequestTest, parsing) {
      myactuator_rmd::SetAccelerationRequest const request {{0x43, 0x03, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00}};
      std::uint32_t const acceleration {request.getAcceleration()};
      AccelerationType const mode {request.getMode()};
      EXPECT_EQ(acceleration, 10000);
      EXPECT_EQ(mode, AccelerationType::VELOCITY_PLANNING_DECELERATION);
    }

    TEST(SetGainToRomRequestTest, parsing) {
      // V4.3: index 0x01 (CURRENT_KP), float value 1.0f = 0x3F800000 (LE: 00 00 80 3F)
      myactuator_rmd::SetGainToRomRequest const request {{0x32, 0x01, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F}};
      auto const index {request.getIndex()};
      auto const value {request.getValue()};
      EXPECT_EQ(index, GainIndex::CURRENT_KP);
      EXPECT_NEAR(value, 1.0f, 0.001f);
    }

    TEST(SetGainToRamRequestTest, parsing) {
      // V4.3: index 0x04 (SPEED_KP), float value 2.5f = 0x40200000 (LE: 00 00 20 40)
      myactuator_rmd::SetGainToRamRequest const request {{0x31, 0x04, 0x00, 0x00, 0x00, 0x00, 0x20, 0x40}};
      auto const index {request.getIndex()};
      auto const value {request.getValue()};
      EXPECT_EQ(index, GainIndex::SPEED_KP);
      EXPECT_NEAR(value, 2.5f, 0.001f);
    }

    TEST(GetGainRequestTest, parsing) {
      // V4.3: read request for index 0x07 (POSITION_KP)
      myactuator_rmd::GetGainRequest const request {{0x30, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}};
      auto const index {request.getIndex()};
      EXPECT_EQ(index, GainIndex::POSITION_KP);
    }

    TEST(SetPositionAbsoluteRequestTest, parsingPositivePosition) {
      myactuator_rmd::SetPositionAbsoluteRequest const request ({0xA4, 0x00, 0xF4, 0x01, 0xA0, 0x8C, 0x00, 0x00});
      auto const position {request.getPosition()};
      EXPECT_NEAR(position, 360.0f, 0.1f);
      auto const max_speed {request.getMaxSpeed()};
      EXPECT_NEAR(max_speed, 500.0f, 0.1f);
    }

    TEST(SetPositionAbsoluteRequestTest, parsingNegativePosition) {
      myactuator_rmd::SetPositionAbsoluteRequest const request ({0xA4, 0x00, 0xF4, 0x01, 0x60, 0x73, 0xFF, 0xFF});
      auto const position {request.getPosition()};
      EXPECT_NEAR(position, -360.0f, 0.1f);
      auto const max_speed {request.getMaxSpeed()};
      EXPECT_NEAR(max_speed, 500.0f, 0.1f);
    }

    TEST(SetTimeoutOffRequestTest, parsing) {
      myactuator_rmd::SetTimeoutRequest const request {{0xB3, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}};
      std::chrono::milliseconds const timeout {request.getTimeout()};
      EXPECT_EQ(timeout.count(), 0);
    }

    TEST(SetTimeoutRequestTest, parsing) {
      myactuator_rmd::SetTimeoutRequest const request {{0xB3, 0x00, 0x00, 0x00, 0xE8, 0x03, 0x00, 0x00}};
      std::chrono::milliseconds const timeout {request.getTimeout()};
      EXPECT_EQ(timeout.count(), 1000);
    }

    TEST(SetTorqueRequestTest, parsingPositiveCurrent) {
      myactuator_rmd::SetTorqueRequest const request ({0xA1, 0x00, 0x00, 0x00, 0x64, 0x00, 0x00, 0x00});
      auto const current {request.getTorqueCurrent()};
      EXPECT_NEAR(current, 1.0f, 0.1f);
    }

    TEST(SetTorqueRequestTest, parsingNegativeCurrent) {
      myactuator_rmd::SetTorqueRequest const request ({0xA1, 0x00, 0x00, 0x00, 0x9C, 0xFF, 0x00, 0x00});
      auto const current {request.getTorqueCurrent()};
      EXPECT_NEAR(current, -1.0f, 0.1f);
    }

    TEST(SetVelocityRequestTest, parsingPositiveVelocity) {
      myactuator_rmd::SetVelocityRequest const request ({0xA2, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00});
      auto const speed {request.getSpeed()};
      EXPECT_NEAR(speed, 100.0f, 0.1f);
    }

    TEST(SetVelocityRequestTest, parsingNegativeVelocity) {
      myactuator_rmd::SetVelocityRequest const request ({0xA2, 0x00, 0x00, 0x00, 0xF0, 0xD8, 0xFF, 0xFF});
      auto const speed {request.getSpeed()};
      EXPECT_NEAR(speed, -100.0f, 0.1f);
    }

  }
}
