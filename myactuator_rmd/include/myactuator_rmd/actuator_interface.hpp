/**
 * \file actuator_interface.hpp
 * \mainpage
 *    Contains the interface to a single actuator
 * \author
 *    Tobit Flatscher (github.com/2b-t)
*/

#ifndef MYACTUATOR_RMD__ACTUATOR_INTERFACE
#define MYACTUATOR_RMD__ACTUATOR_INTERFACE
#pragma once

#include <chrono>
#include <cstdint>
#include <string>

#include "myactuator_rmd/actuator_state/acceleration_type.hpp"
#include "myactuator_rmd/actuator_state/can_baud_rate.hpp"
#include "myactuator_rmd/actuator_state/control_mode.hpp"
#include "myactuator_rmd/actuator_state/feedback.hpp"
#include "myactuator_rmd/actuator_state/gain_index.hpp"
#include "myactuator_rmd/actuator_state/gains.hpp"
#include "myactuator_rmd/actuator_state/motor_status_1.hpp"
#include "myactuator_rmd/actuator_state/motor_status_2.hpp"
#include "myactuator_rmd/actuator_state/motor_status_3.hpp"
#include "myactuator_rmd/driver/driver.hpp"


namespace myactuator_rmd {

  /**\class ActuatorInterface
   * \brief
   *    Actuator for commanding the MyActuator RMD actuator series
  */
  class ActuatorInterface {
    public:
      ActuatorInterface(Driver& driver, std::uint32_t const actuator_id);
      ActuatorInterface() = delete;
      ActuatorInterface(ActuatorInterface const&) = default;
      ActuatorInterface& operator = (ActuatorInterface const&) = default;
      ActuatorInterface(ActuatorInterface&&) = default;
      ActuatorInterface& operator = (ActuatorInterface&&) = default;

      /**\fn getAcceleration
       * \brief
       *    Reads the current acceleration
       *
       * \return
       *    The current acceleration in dps with a resolution of 1 dps
      */
      [[nodiscard]]
      std::int32_t getAcceleration();

      /**\fn getCanId
       * \brief
       *    Get the CAN ID of the device
       *
       * \return
       *    The CAN ID of the device
      */
      [[nodiscard]]
      std::uint16_t getCanId();

      /**\fn getControllerGains
       * \brief
       *    Reads all PID controller gains (V4.3: 9 individual read requests)
       *
       * \return
       *    The currently used PID gains for current, speed, and position loops
      */
      [[nodiscard]]
      Gains getControllerGains();

      /**\fn getGain
       * \brief
       *    Read a single PID gain parameter by index (V4.3)
       *
       * \param[in] index
       *    The gain parameter index to read
       * \return
       *    The gain parameter value as a float
      */
      [[nodiscard]]
      float getGain(GainIndex const index);

      /**\fn getControlMode
       * \brief
       *    Reads the currently used control mode
       *
       * \return
       *    The currently used control mode
      */
      [[nodiscard]]
      ControlMode getControlMode();

      [[nodiscard]]
      std::string getMotorModel();

      [[nodiscard]]
      float getMotorPower();

      [[nodiscard]]
      MotorStatus1 getMotorStatus1();

      [[nodiscard]]
      MotorStatus2 getMotorStatus2();

      [[nodiscard]]
      MotorStatus3 getMotorStatus3();

      [[nodiscard]]
      float getMultiTurnAngle();

      [[nodiscard]]
      std::int32_t getMultiTurnEncoderPosition();

      [[nodiscard]]
      std::int32_t getMultiTurnEncoderOriginalPosition();

      [[nodiscard]]
      std::int32_t getMultiTurnEncoderZeroOffset();

      [[nodiscard]]
      std::chrono::milliseconds getRuntime();

      [[nodiscard]]
      float getSingleTurnAngle();

      [[nodiscard]]
      std::int16_t getSingleTurnEncoderPosition();

      [[nodiscard]]
      std::uint32_t getVersionDate();

      void lockBrake();

      void releaseBrake();

      void reset();

      /**\fn sendCurrentSetpoint
       * \brief
       *    Send a current set-point to the actuator
       *
       * \param[in] current
       *    The current set-point in Ampere
       * \return
       *    Feedback control message
      */
      Feedback sendCurrentSetpoint(float const current);

      /**\fn sendPositionAbsoluteSetpoint
       * \brief
       *    Send an absolute position set-point to the actuator
       *
       * \param[in] position
       *    The position set-point in degree
       * \param[in] max_speed
       *    The maximum speed for the motion in degree per second
       * \return
       *    Feedback control message
      */
      Feedback sendPositionAbsoluteSetpoint(float const position, float const max_speed = 500.0);

      /**\fn sendForcePositionSetpoint
       * \brief
       *    Send a force-controlled position command (V4.3 0xA9) with torque limiting
       *
       * \param[in] position
       *    The position set-point in degree
       * \param[in] max_speed
       *    The maximum speed in degree per second
       * \param[in] max_torque
       *    Maximum torque as percentage of rated current (0-255, 1 = 1%)
       * \return
       *    Feedback control message
      */
      Feedback sendForcePositionSetpoint(float const position, float const max_speed, std::uint8_t const max_torque);

      /**\fn sendSingleTurnPositionSetpoint
       * \brief
       *    Send a single-turn position command (0xA6)
       *
       * \param[in] position
       *    The position set-point in degrees (0-359.99)
       * \param[in] max_speed
       *    The maximum speed in degree per second
       * \param[in] direction
       *    Spin direction: 0x00 = clockwise, 0x01 = counter-clockwise
       * \return
       *    Feedback control message
      */
      Feedback sendSingleTurnPositionSetpoint(float const position, float const max_speed, std::uint8_t const direction);

      /**\fn sendIncrementalPositionSetpoint
       * \brief
       *    Send an incremental (relative) position command (0xA8)
       *
       * \param[in] angle_increment
       *    Relative angle increment in degrees
       * \param[in] max_speed
       *    The maximum speed in degree per second
       * \return
       *    Feedback control message
      */
      Feedback sendIncrementalPositionSetpoint(float const angle_increment, float const max_speed);

      /**\fn sendTorqueSetpoint
       * \brief
       *    Send a torque set-point to the actuator by setting the current
       *
       * \param[in] torque
       *    The desired torque in [Nm]
       * \param[in] torque_constant
       *    The motor's torque constant [Nm/A]
       * \return
       *    Feedback control message
      */
      Feedback sendTorqueSetpoint(float const torque, float const torque_constant);

      /**\fn sendVelocitySetpoint
       * \brief
       *    Send a velocity set-point to the actuator
       *
       * \param[in] speed
       *    The speed set-point in degree per second
       * \param[in] max_torque
       *    Maximum torque as percentage of rated current (0-255, 1 = 1%), 0 = no limit
       * \return
       *    Feedback control message
      */
      Feedback sendVelocitySetpoint(float const speed, std::uint8_t const max_torque = 0);

      void setAcceleration(std::uint32_t const acceleration, AccelerationType const mode);

      void setCanBaudRate(CanBaudRate const baud_rate);

      void setCanId(std::uint16_t const can_id);

      std::int32_t setCurrentPositionAsEncoderZero();

      void setEncoderZero(std::int32_t const encoder_offset);

      /**\fn setControllerGains
       * \brief
       *    Write all PID controller gains (V4.3: 9 individual write requests)
       *
       * \param[in] gains
       *    The PID gains for current, speed, and position to be set
       * \param[in] is_persistent
       *    Whether gains persist after reboot (ROM) or not (RAM)
       * \return
       *    The confirmed controller gains
      */
      Gains setControllerGains(Gains const& gains, bool const is_persistent = false);

      /**\fn setGain
       * \brief
       *    Write a single PID gain parameter by index (V4.3)
       *
       * \param[in] index
       *    The gain parameter index to write
       * \param[in] value
       *    The gain value as a float
       * \param[in] is_persistent
       *    Whether the gain persists after reboot
       * \return
       *    The confirmed gain value
      */
      float setGain(GainIndex const index, float const value, bool const is_persistent = false);

      void setTimeout(std::chrono::milliseconds const& timeout);

      void shutdownMotor();

      void stopMotor();

    protected:
      Driver& driver_;
      std::uint32_t actuator_id_;
  };

}

#endif // MYACTUATOR_RMD__ACTUATOR_INTERFACE
