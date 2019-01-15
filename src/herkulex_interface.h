#ifndef __HERKULEX_INTERFACE_H__
#define __HERKULEX_INTERFACE_H__

#include <math.h>
#include <stdlib.h>

#include <boost/asio.hpp>
#include <boost/thread.hpp>

// ROS libraries
#include "ros/ros.h"

#include "blocking_reader.h"

// ----- Driver Constants from datasheet --------------------------------------------------
#define EEP_WRITE (u_char) 0x01 // command for writing to EEP
#define EEP_READ (u_char) 0x02// command for reading from EEP
#define RAM_WRITE (u_char) 0x03 // command for writing to RAM
#define RAM_READ (u_char) 0x04 // command for reading from RAM
#define I_JOG (u_char) 0x05 // command for servo movement control
#define S_JOG (u_char) 0x06 // command for servo movement control
#define STAT (u_char) 0x07 // command for getting servo status
#define ROLLBACK (u_char) 0x08 // command for restoring factory defaults
#define REBOOT 0x09 // command for rebooting servo

#define ACK_POLICY_RAM_ADDR (u_char) 0x01
#define ACK_POLICY_EEP_ADDR (u_char) 0x07

#define ACK_POLICY_NO_REPLY (u_char) 0x00
#define ACK_POLICY_REPLY_TO_READ (u_char) 0x01
#define ACK_POLICY_REPLY_TO_ALL (u_char) 0x02

#define TORQUE_CONTROL_RAM_ADDR (u_char) 52
#define TORQUE_CONTROL_TORQUE_FREE (u_char) 0x00 // servo manually movable, operation command (I_JOG, S_JOG) not possible
#define TORQUE_CONTROL_BREAK_ON (u_char) 0x40 // servo stopped, operation command (I_JOG, S_JOG) not possible
#define TORQUE_CONTROL_TORQUE_ON (u_char) 0x60 // operation possible

#define RAM_ADDR_VOLTATE		54
#define RAM_ADDR_TEMPERATURE	55

#define VOLTAGE_EEP_REG		49
#define TEMPERATURE_EEP_REG	50

#define LED_CONTROL_RAM_ADDR (u_char) 53
#define LED_CONTROL_OFF (u_char) 0x00
#define LED_CONTROL_GREEN (u_char) 0x01
#define LED_CONTROL_BLUE (u_char) 0x02
#define LED_CONTROL_RED (u_char) 0x04

#define STATUS_ERROR_RAM_ADDR (u_char) 48

#define STATUS_ERROR_VOLTAGE_EXCEEDED		0x01
#define STATUS_ERROR_RANGE_EXCEEDED			0x02
#define STATUS_ERROR_TEMPERATURE_EXCEEDED	0x04
#define STATUS_ERROR_INVALID_PACKET			0x08
#define STATUS_ERROR_OVERLOAD_DETECTED		0x10
#define STATUS_ERROR_EEG_REGISTER_DISTORTED 0x40

#define STATUS_DETAIL_MOVING				0x01
#define STATUS_DETAIL_IN_POSITION			0x02
#define STATUS_DETAIL_CHECKSUM_ERROR		0x04
#define STATUS_DETAIL_UNKNOWN_COMMAND		0x08
#define STATUS_DETAIL_EXCEEDED_REG_RANGE	0x10
#define STATUS_DETAIL_GARBAGE_DETECTED		0x20
#define STATUS_DETAIL_TORQUE_ON				0x40

#define MIN_POSITION_RAM_ADDR (u_char) 20
#define MIN_POSITION_EEP_ADDR (u_char) 26
#define MAX_POSITION_RAM_ADDR (u_char) 22
#define MAX_POSITION_EEP_ADDR (u_char) 28

#define ABSOLUTE_POSITION_RAM_ADDR (u_char) 60

#define SERVO_TYPE_UNKNOWN 0
#define SERVO_TYPE_DRS_0101 1
#define SERVO_TYPE_DRS_0602 2

#define DRS_0101_MIN_POS 0 // steps
#define DRS_0101_MAX_POS 1023 // steps
#define DRS_0101_RESOLUTION 0.325 // degrees/step
#define DRS_0101_ZERO_POS 512 // steps

#define DRS_0602_MIN_POS 10381 // steps
#define DRS_0602_MAX_POS  22129 // steps
#define DRS_0602_RESOLUTION 0.02778 // degrees/step
#define DRS_0602_ZERO_POS 16384 // steps
// ----------------------------------------------------------------------------------------

namespace Herkulex
{
	class Servo;
	class TrajectoryPoint;
	class Trajectory;
	class ServoJointStatus;
	class ServoPowerStatus;
	class Interface;

	enum ShutdownPolicy { Brake, Slack };
};

class Herkulex::Servo
{
public:
	Servo()
	{
		angleMin = -2.7855455;
		angleMax = 2.7855455;
		interface = NULL;
	}
	Servo(u_char id, std::string type, std::string firmware, Herkulex::Interface *interface)
	{
		this->id = id;
		this->type = type;
		this->firmwareVersion = firmware;
		this->interface = interface;

		angleMin = -2.7855455;
		angleMax = 2.7855455;
	}

	u_char id;
	std::string type;
	std::string firmwareVersion;

	float angleMin, angleMax;

	Herkulex::Interface *interface;

	std::string toString()
	{
		char string[50];
		sprintf(string, "Id : %03d - '%s' firmware %s",
				id,
				type.c_str(),
				firmwareVersion.c_str());
		return std::string(string);
	}

	void reboot();
};

class Herkulex::TrajectoryPoint
{
public:
	TrajectoryPoint(int id, float time, float angle) :
		servoId(id),
		timeFromStartSecs(time),
		angle(angle) {};

	int servoId;
	float timeFromStartSecs;
	float angle;

	void setDegrees(float angleDegrees) { angle = (angleDegrees/180) * 3.141592654; };
};

class Herkulex::Trajectory
{
public:

	// The outer vector is the points in time
	// and the inner vectors are the position of each joint
	// at that point in time.
	std::vector<std::vector<TrajectoryPoint> > points;
};

class Herkulex::ServoJointStatus
{
public:
	bool known;
	int servoId;
	float angleRads;
	float velocityRadsPerSec;
	float torqueNm;
	u_char error;
	u_char detail;
};

class Herkulex::ServoPowerStatus
{
public:
	float voltageVolts;
	int tempDegrees;
};

class Herkulex::Interface
{
public:
	Interface(std::string portName, int BAUD = 115200);
	~Interface();

	std::vector<Herkulex::Servo> getServos() { return servos; };

	bool isConnected() { return connected; };
	std::string getConnectionError() { return connectionError; };

	void setShudownPolicy(Herkulex::ShutdownPolicy newPolicy) { shutdownPolicy = newPolicy; };

	bool jogServo(Herkulex::TrajectoryPoint position);
	bool jogServos(std::vector<Herkulex::TrajectoryPoint> positions);

	bool followTrajectory(Herkulex::Trajectory);

	Herkulex::ServoJointStatus getJointState(u_char servoId);
	std::vector<Herkulex::ServoJointStatus> getJointStates();

	Herkulex::ServoPowerStatus getServoPowerStatus(u_char servoId);
	std::vector<Herkulex::ServoPowerStatus> getServoPowerStatuses();

	// Servo configuration methods
	void setLed(u_char servoId, u_char ledState);
	void setTorqueMode(u_char servoId, u_char torqueMode);

	static std::string getErrorString(u_char statusError, u_char statusDetail = 0);
	static std::string getDetailString(u_char statusDetail);

	std::vector<Herkulex::Servo> servos;

private:
	friend class Herkulex::Servo;

	// Helper functions
	std::vector<u_char> assembleBytes(u_char b1, u_char b2);
	std::vector<u_char> assembleBytes(u_char b1, u_char b2, u_char b3);

	// Methods encapsulating the equations to convert radians to and from raw angle values
	// This also limits the radians to raw calculation to within the safe range
	// This should also do something cleverer in the future where the model of the servo is
	// use to automatically determine the equation.
	short radiansToRaw(u_char servoId, float radians);
	float rawToRadians(u_char servoId, short raw);

	// Internal protocol functions
	std::vector<u_char> makeCommandPacket(u_char servoId, u_char command, std::vector<u_char> data);
	std::vector<u_char> getAckResponseFull(u_char cmd, int timeout = 500);
	std::vector<u_char> getAckResponse(u_char cmd, int timeout = 500);

	std::vector<u_char> ramRead(u_char servoId, u_char addr, u_char length);
	void ramWrite(u_char servoId, std::vector<u_char> data);

	std::vector<u_char> eepReadFull(u_char servo_id, u_char addr, u_char len, int timeout = 100);
	std::vector<u_char> eepRead(u_char servo_id, u_char addr, u_char len, int timeout = 100);

	// High level internal functions
	void detectServos();

	std::vector<u_char> makeSJOGPacket(Herkulex::TrajectoryPoint position, u_char ledStatus);
	void S_JOGCommand(u_char servoId, std::vector<u_char> packets);

	Herkulex::Servo *getServoById(u_char servoId);

	boost::asio::io_service io;
	boost::asio::serial_port *port;
	bool connected;
	std::string connectionError;

	Herkulex::ShutdownPolicy shutdownPolicy;
};

#endif










