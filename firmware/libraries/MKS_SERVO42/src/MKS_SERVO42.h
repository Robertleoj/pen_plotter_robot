/// \file MKS_SERVO42.h
/// \brief Provides control over MKS SERVO42 stepper motors via serial communication.
///
/// \details This class is designed to interface with MKS SERVO42 stepper motors, allowing for
///  		 initialization and control of the motors through a series of commands sent over a
///  		 serial connection. It provides methods to ping motors to check their status, retrieve
///  		 current position, and set target positions with specified speed and direction.
///  		 It encapsulates the communication protocol needed to interact with the motors, providing
///  		 a higher-level interface for ease of use in applications.
///          See https://github.com/makerbase-mks/MKS-SERVO42C/wiki/Serial-communication-description-V1.0 for documentation

#pragma once
#include <Arduino.h>

namespace Instruction
{
	byte const GET_ENCODER_POS = 0x30;
	byte const STEPPER_PING = 0x3A;
	byte const MOVE_SPEED_PULSES = 0xFD;
	byte const SET_SPEED_DIRECTION = 0xF6;
	byte const SET_SPEED = 0xF3;
	byte const SET_ZERO_MODE = 0x90;
	byte const SET_ZERO_POS = 0x91;
	byte const SET_ZERO_SPEED = 0x92;
	byte const SET_SUBDIVISION = 0x84;
};

enum class ZeroMode
{
	OFF = 0x00,
	DIRMODE = 0x01,
	NEARMODE = 0x02
};

enum class FirmwareVersion
{
	V1_0,
	V1_1
};

/// @brief Driver for MKS SERVO42 stepper motors via serial communication
class MKS_SERVO42
{
public:
	/**
	 * @brief Initializes the MKS_SERVO42 object with a serial port and a baud rate.
	 * @param serialPort Pointer to the HardwareSerial object that corresponds to the serial port.
	 * @param stepperId The ID of the stepper motor to be controlled.
	 * @param baudRate The baud rate for serial communication.
	 * @param version The version of the MKS_SERVO42 motor controller.
	 */

	void initialize(HardwareSerial *serialPort, byte stepperId, FirmwareVersion const &version);
	// void initialize(SoftwareSerial *serialPort = nullptr, long const &baudRate = 38400);


	/// @brief Sends a ping command to the stepper motor to check its presence and connectivity.
	/// @param stepperId The ID of the stepper motor to ping.
	/// @return Returns true if the stepper motor is successfully pinged, false otherwise.
	bool ping();

	/// @brief Retrieves the current position of the stepper motor in pulses.
	/// @return Returns the current position of the stepper motor as a long integer value.
	long getCurrentPosition();

	/// @brief Sets the target position for the stepper motor by specifying the direction, speed, and number of pulses.
	///        The speed parameter should not exceed 2000 RPM and the direction should be 0 for one way or 1 for the opposite.
	/// @param direction The direction in which the motor should run (0 or 1).
	/// @param speed The speed at which the motor should run, up to a maximum of 2000 RPM.
	/// @param pulses The number of pulses to move, which translates to the target position.
	/// @return Returns true if the command is successfully sent and the motor starts running, false if there is an error.
	bool setTargetPosition(byte direction, uint8_t speed, uint32_t pulses);
	bool setSpeedDirection(uint8_t speed, byte direction);

	bool setZeroPosition();
	bool setZeroSpeed(uint8_t speed);
	bool setZeroMode(ZeroMode mode);
	bool setSubdivision(uint8_t subdivision);

private:
	HardwareSerial *port = nullptr;
	FirmwareVersion version;
	byte stepperId;

	bool sendMessage(byte const &commandID);
	int receiveStepperStatus();
	long receiveEncoderPosition();
	byte calculateChecksum(const byte *message, int length);
	void flush();
	byte padStepperId(byte const &stepperId);
};