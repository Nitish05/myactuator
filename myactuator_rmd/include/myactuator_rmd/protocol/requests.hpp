/**
 * \file requests.hpp
 * \mainpage
 *    Contains all the requests sent from the driver to the actuator
 * \author
 *    Tobit Flatscher (github.com/2b-t)
*/

#ifndef MYACTUATOR_RMD__PROTOCOL__REQUESTS
#define MYACTUATOR_RMD__PROTOCOL__REQUESTS
#pragma once

#include <chrono>
#include <cstdint>

#include "myactuator_rmd/actuator_state/acceleration_type.hpp"
#include "myactuator_rmd/actuator_state/can_baud_rate.hpp"
#include "myactuator_rmd/actuator_state/gain_index.hpp"
#include "myactuator_rmd/actuator_state/gains.hpp"
#include "myactuator_rmd/protocol/command_type.hpp"
#include "myactuator_rmd/protocol/single_motor_message.hpp"


namespace myactuator_rmd {

  using GetAccelerationRequest = SingleMotorRequest<CommandType::READ_ACCELERATION>;
  using GetControlModeRequest = SingleMotorRequest<CommandType::READ_SYSTEM_OPERATING_MODE>;
  using GetMotorModelRequest =  SingleMotorRequest<CommandType::READ_MOTOR_MODEL>;
  using GetMotorPowerRequest =  SingleMotorRequest<CommandType::READ_MOTOR_POWER>;
  using GetMotorStatus1Request = SingleMotorRequest<CommandType::READ_MOTOR_STATUS_1_AND_ERROR_FLAG>;
  using GetMotorStatus2Request =  SingleMotorRequest<CommandType::READ_MOTOR_STATUS_2>;
  using GetMotorStatus3Request = SingleMotorRequest<CommandType::READ_MOTOR_STATUS_3>;
  using GetMultiTurnAngleRequest = SingleMotorRequest<CommandType::READ_MULTI_TURN_ANGLE>;
  using GetMultiTurnEncoderPositionRequest = SingleMotorRequest<CommandType::READ_MULTI_TURN_ENCODER_POSITION>;
  using GetMultiTurnEncoderOriginalPositionRequest = SingleMotorRequest<CommandType::READ_MULTI_TURN_ENCODER_ORIGINAL_POSITION>;
  using GetMultiTurnEncoderZeroOffsetRequest = SingleMotorRequest<CommandType::READ_MULTI_TURN_ENCODER_ZERO_OFFSET>;
  using GetSingleTurnAngleRequest = SingleMotorRequest<CommandType::READ_SINGLE_TURN_ANGLE>;
  using GetSingleTurnEncoderPositionRequest = SingleMotorRequest<CommandType::READ_SINGLE_TURN_ENCODER>;
  using GetSystemRuntimeRequest = SingleMotorRequest<CommandType::READ_SYSTEM_RUNTIME>;
  using GetVersionDateRequest = SingleMotorRequest<CommandType::READ_SYSTEM_SOFTWARE_VERSION_DATE>;
  using LockBrakeRequest = SingleMotorRequest<CommandType::LOCK_BRAKE>;
  using ReleaseBrakeRequest = SingleMotorRequest<CommandType::RELEASE_BRAKE>;
  using ResetRequest = SingleMotorRequest<CommandType::RESET_SYSTEM>;
  using SetCurrentPositionAsEncoderZeroRequest = SingleMotorRequest<CommandType::WRITE_CURRENT_MULTI_TURN_POSITION_TO_ROM_AS_ZERO>;

  /**\class CanIdRequest
   * \brief
   *    Request for getting/setting the CAN ID of the actuator
  */
  class CanIdRequest: public SingleMotorRequest<CommandType::CAN_ID_SETTING> {
    public:
      using SingleMotorRequest::SingleMotorRequest;

      [[nodiscard]]
      bool isWrite() const noexcept;

    protected:
      CanIdRequest() = default;
      CanIdRequest(CanIdRequest const&) = default;
      CanIdRequest& operator = (CanIdRequest const&) = default;
      CanIdRequest(CanIdRequest&&) = default;
      CanIdRequest& operator = (CanIdRequest&&) = default;
  };

  /**\class GetCanIdRequest
   * \brief
   *    Request for getting the CAN ID of the actuator
  */
  class GetCanIdRequest: public CanIdRequest {
    public:
      GetCanIdRequest();
      GetCanIdRequest(GetCanIdRequest const&) = default;
      GetCanIdRequest& operator = (GetCanIdRequest const&) = default;
      GetCanIdRequest(GetCanIdRequest&&) = default;
      GetCanIdRequest& operator = (GetCanIdRequest&&) = default;
      using CanIdRequest::CanIdRequest;
  };

  /**\class SetCanIdRequest
   * \brief
   *    Request for setting the CAN ID of the actuator
  */
  class SetCanIdRequest: public CanIdRequest {
    public:
      SetCanIdRequest(std::uint16_t const can_id);
      SetCanIdRequest(SetCanIdRequest const&) = default;
      SetCanIdRequest& operator = (SetCanIdRequest const&) = default;
      SetCanIdRequest(SetCanIdRequest&&) = default;
      SetCanIdRequest& operator = (SetCanIdRequest&&) = default;
      using CanIdRequest::CanIdRequest;

      [[nodiscard]]
      std::uint16_t getCanId() const noexcept;
  };

  /**\class SetAccelerationRequest
   * \brief
   *    Request for setting the maximum acceleration/deceleration of the actuator
  */
  class SetAccelerationRequest: public SingleMotorRequest<CommandType::WRITE_ACCELERATION_TO_RAM_AND_ROM> {
    public:
      SetAccelerationRequest(std::uint32_t const acceleration, AccelerationType const mode);
      SetAccelerationRequest() = delete;
      SetAccelerationRequest(SetAccelerationRequest const&) = default;
      SetAccelerationRequest& operator = (SetAccelerationRequest const&) = default;
      SetAccelerationRequest(SetAccelerationRequest&&) = default;
      SetAccelerationRequest& operator = (SetAccelerationRequest&&) = default;
      using SingleMotorRequest::SingleMotorRequest;

      [[nodiscard]]
      std::uint32_t getAcceleration() const noexcept;

      [[nodiscard]]
      AccelerationType getMode() const noexcept;
  };

  /**\class SetCanBaudRateRequest
   * \brief
   *    Request for setting the Baud rate of the actuator
  */
  class SetCanBaudRateRequest: public SingleMotorRequest<CommandType::COMMUNICATION_BAUD_RATE_SETTING> {
    public:
      SetCanBaudRateRequest(CanBaudRate const baud_rate);
      SetCanBaudRateRequest(SetCanBaudRateRequest const&) = default;
      SetCanBaudRateRequest& operator = (SetCanBaudRateRequest const&) = default;
      SetCanBaudRateRequest(SetCanBaudRateRequest&&) = default;
      SetCanBaudRateRequest& operator = (SetCanBaudRateRequest&&) = default;
      using SingleMotorRequest::SingleMotorRequest;

      [[nodiscard]]
      CanBaudRate getBaudRate() const noexcept;
  };

  /**\class SetEncoderZeroRequest
   * \brief
   *    Request for setting the encoder zero to a given value
  */
  class SetEncoderZeroRequest: public SingleMotorRequest<CommandType::WRITE_ENCODER_MULTI_TURN_VALUE_TO_ROM_AS_ZERO> {
    public:
      SetEncoderZeroRequest(std::int32_t const encoder_offset);
      SetEncoderZeroRequest(SetEncoderZeroRequest const&) = default;
      SetEncoderZeroRequest& operator = (SetEncoderZeroRequest const&) = default;
      SetEncoderZeroRequest(SetEncoderZeroRequest&&) = default;
      SetEncoderZeroRequest& operator = (SetEncoderZeroRequest&&) = default;
      using SingleMotorRequest::SingleMotorRequest;

      [[nodiscard]]
      std::int32_t getEncoderZero() const noexcept;
  };

  /**\class GetGainRequest
   * \brief
   *    V4.3 request for reading a single PID gain parameter by index
  */
  class GetGainRequest: public SingleMotorRequest<CommandType::READ_PID_PARAMETERS> {
    public:
      GetGainRequest(GainIndex const index);
      GetGainRequest() = delete;
      GetGainRequest(GetGainRequest const&) = default;
      GetGainRequest& operator = (GetGainRequest const&) = default;
      GetGainRequest(GetGainRequest&&) = default;
      GetGainRequest& operator = (GetGainRequest&&) = default;
      using SingleMotorRequest::SingleMotorRequest;

      [[nodiscard]]
      GainIndex getIndex() const noexcept;
  };

  /**\class SetGainRequest
   * \brief
   *    V4.3 request for writing a single PID gain parameter by index
   *
   * \tparam C
   *    Command type (WRITE_PID_PARAMETERS_TO_RAM or WRITE_PID_PARAMETERS_TO_ROM)
  */
  template <CommandType C>
  class SetGainRequest: public SingleMotorRequest<C> {
    public:
      SetGainRequest(GainIndex const index, float const value);
      SetGainRequest() = delete;
      SetGainRequest(SetGainRequest const&) = default;
      SetGainRequest& operator = (SetGainRequest const&) = default;
      SetGainRequest(SetGainRequest&&) = default;
      SetGainRequest& operator = (SetGainRequest&&) = default;
      using SingleMotorRequest<C>::SingleMotorRequest;

      [[nodiscard]]
      GainIndex getIndex() const noexcept;

      [[nodiscard]]
      float getValue() const noexcept;
  };

  template <CommandType C>
  SetGainRequest<C>::SetGainRequest(GainIndex const index, float const value)
  : SingleMotorRequest<C>{} {
    this->data_[1] = static_cast<std::uint8_t>(index);
    this->template setAt<float>(value, 4);
    return;
  }

  template <CommandType C>
  GainIndex SetGainRequest<C>::getIndex() const noexcept {
    return static_cast<GainIndex>(this->data_[1]);
  }

  template <CommandType C>
  float SetGainRequest<C>::getValue() const noexcept {
    return this->template getAs<float>(4);
  }

  using SetGainToRamRequest = SetGainRequest<CommandType::WRITE_PID_PARAMETERS_TO_RAM>;
  using SetGainToRomRequest = SetGainRequest<CommandType::WRITE_PID_PARAMETERS_TO_ROM>;

  /**\class SetPositionAbsoluteRequest
   * \brief
   *    Request for setting the absolute position of the actuator with a given maximum speed
  */
  class SetPositionAbsoluteRequest: public SingleMotorRequest<CommandType::ABSOLUTE_POSITION_CLOSED_LOOP_CONTROL> {
    public:
      SetPositionAbsoluteRequest(float const position, float const max_speed);
      SetPositionAbsoluteRequest() = delete;
      SetPositionAbsoluteRequest(SetPositionAbsoluteRequest const&) = default;
      SetPositionAbsoluteRequest& operator = (SetPositionAbsoluteRequest const&) = default;
      SetPositionAbsoluteRequest(SetPositionAbsoluteRequest&&) = default;
      SetPositionAbsoluteRequest& operator = (SetPositionAbsoluteRequest&&) = default;
      using SingleMotorRequest::SingleMotorRequest;

      [[nodiscard]]
      float getMaxSpeed() const noexcept;

      [[nodiscard]]
      float getPosition() const noexcept;
  };

  /**\class SetForcePositionRequest
   * \brief
   *    V4.3 request for position control with torque limiting (0xA9)
  */
  class SetForcePositionRequest: public SingleMotorRequest<CommandType::FORCE_POSITION_CLOSED_LOOP_CONTROL> {
    public:
      /**\fn SetForcePositionRequest
       * \brief
       *    Class constructor
       *
       * \param[in] position
       *    The position set-point in degrees
       * \param[in] max_speed
       *    The maximum speed in degrees per second
       * \param[in] max_torque
       *    Maximum torque as percentage of rated current (0-255, 1 = 1%)
      */
      SetForcePositionRequest(float const position, float const max_speed, std::uint8_t const max_torque);
      SetForcePositionRequest() = delete;
      SetForcePositionRequest(SetForcePositionRequest const&) = default;
      SetForcePositionRequest& operator = (SetForcePositionRequest const&) = default;
      SetForcePositionRequest(SetForcePositionRequest&&) = default;
      SetForcePositionRequest& operator = (SetForcePositionRequest&&) = default;
      using SingleMotorRequest::SingleMotorRequest;

      [[nodiscard]]
      float getPosition() const noexcept;

      [[nodiscard]]
      float getMaxSpeed() const noexcept;

      [[nodiscard]]
      std::uint8_t getMaxTorque() const noexcept;
  };

  /**\class SetSingleTurnPositionRequest
   * \brief
   *    Request for single-turn position control (0xA6)
  */
  class SetSingleTurnPositionRequest: public SingleMotorRequest<CommandType::SINGLE_TURN_POSITION_CONTROL> {
    public:
      /**\fn SetSingleTurnPositionRequest
       * \brief
       *    Class constructor
       *
       * \param[in] position
       *    The position set-point in degrees (0-359.99)
       * \param[in] max_speed
       *    The maximum speed in degrees per second
       * \param[in] direction
       *    Spin direction: 0x00 = clockwise, 0x01 = counter-clockwise
      */
      SetSingleTurnPositionRequest(float const position, float const max_speed, std::uint8_t const direction);
      SetSingleTurnPositionRequest() = delete;
      SetSingleTurnPositionRequest(SetSingleTurnPositionRequest const&) = default;
      SetSingleTurnPositionRequest& operator = (SetSingleTurnPositionRequest const&) = default;
      SetSingleTurnPositionRequest(SetSingleTurnPositionRequest&&) = default;
      SetSingleTurnPositionRequest& operator = (SetSingleTurnPositionRequest&&) = default;
      using SingleMotorRequest::SingleMotorRequest;

      [[nodiscard]]
      float getPosition() const noexcept;

      [[nodiscard]]
      float getMaxSpeed() const noexcept;

      [[nodiscard]]
      std::uint8_t getDirection() const noexcept;
  };

  /**\class SetIncrementalPositionRequest
   * \brief
   *    Request for incremental position control (0xA8)
  */
  class SetIncrementalPositionRequest: public SingleMotorRequest<CommandType::INCREMENTAL_POSITION_CLOSED_LOOP_CONTROL> {
    public:
      /**\fn SetIncrementalPositionRequest
       * \brief
       *    Class constructor
       *
       * \param[in] angle_increment
       *    Relative angle increment in degrees (positive or negative)
       * \param[in] max_speed
       *    The maximum speed in degrees per second
      */
      SetIncrementalPositionRequest(float const angle_increment, float const max_speed);
      SetIncrementalPositionRequest() = delete;
      SetIncrementalPositionRequest(SetIncrementalPositionRequest const&) = default;
      SetIncrementalPositionRequest& operator = (SetIncrementalPositionRequest const&) = default;
      SetIncrementalPositionRequest(SetIncrementalPositionRequest&&) = default;
      SetIncrementalPositionRequest& operator = (SetIncrementalPositionRequest&&) = default;
      using SingleMotorRequest::SingleMotorRequest;

      [[nodiscard]]
      float getAngleIncrement() const noexcept;

      [[nodiscard]]
      float getMaxSpeed() const noexcept;
  };

  /**\class SetTimeoutRequest
   * \brief
   *    Request for setting the communication interruption protection time setting
  */
  class SetTimeoutRequest: public SingleMotorRequest<CommandType::COMMUNICATION_INTERRUPTION_PROTECTION_TIME_SETTING> {
    public:
      SetTimeoutRequest(std::chrono::milliseconds const& timeout);
      SetTimeoutRequest() = delete;
      SetTimeoutRequest(SetTimeoutRequest const&) = default;
      SetTimeoutRequest& operator = (SetTimeoutRequest const&) = default;
      SetTimeoutRequest(SetTimeoutRequest&&) = default;
      SetTimeoutRequest& operator = (SetTimeoutRequest&&) = default;
      using SingleMotorRequest::SingleMotorRequest;

      [[nodiscard]]
      std::chrono::milliseconds getTimeout() const noexcept;
  };

  /**\class SetTorqueRequest
   * \brief
   *    Request for setting the torque of the actuator by setting a target current
  */
  class SetTorqueRequest: public SingleMotorRequest<CommandType::TORQUE_CLOSED_LOOP_CONTROL> {
    public:
      SetTorqueRequest(float const current);
      SetTorqueRequest() = delete;
      SetTorqueRequest(SetTorqueRequest const&) = default;
      SetTorqueRequest& operator = (SetTorqueRequest const&) = default;
      SetTorqueRequest(SetTorqueRequest&&) = default;
      SetTorqueRequest& operator = (SetTorqueRequest&&) = default;
      using SingleMotorRequest::SingleMotorRequest;

      [[nodiscard]]
      float getTorqueCurrent() const noexcept;
  };

  /**\class SetVelocityRequest
   * \brief
   *    Request for setting the velocity of the actuator
  */
  class SetVelocityRequest: public SingleMotorRequest<CommandType::SPEED_CLOSED_LOOP_CONTROL> {
    public:
      /**\fn SetVelocityRequest
       * \brief
       *    Class constructor
       *
       * \param[in] speed
       *    The velocity set-point in degree per second
       * \param[in] max_torque
       *    Maximum torque as percentage of rated current (0-255, 1 = 1%), 0 = no limit
      */
      SetVelocityRequest(float const speed, std::uint8_t const max_torque = 0);
      SetVelocityRequest() = delete;
      SetVelocityRequest(SetVelocityRequest const&) = default;
      SetVelocityRequest& operator = (SetVelocityRequest const&) = default;
      SetVelocityRequest(SetVelocityRequest&&) = default;
      SetVelocityRequest& operator = (SetVelocityRequest&&) = default;
      using SingleMotorRequest::SingleMotorRequest;

      [[nodiscard]]
      float getSpeed() const noexcept;

      [[nodiscard]]
      std::uint8_t getMaxTorque() const noexcept;
  };

  using ShutdownMotorRequest = SingleMotorRequest<CommandType::SHUTDOWN_MOTOR>;
  using StopMotorRequest = SingleMotorRequest<CommandType::STOP_MOTOR>;

}

#endif // MYACTUATOR_RMD__PROTOCOL__REQUESTS
