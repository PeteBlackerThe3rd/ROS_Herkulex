
#include <math.h>
#include <stdlib.h>
//#include <boost/algorithm/string.hpp>

// ROS libraries
#include "ros/ros.h"
//#include "sensor_msgs/JointState.h"
//#include <actionlib/server/simple_action_server.h>
//#include "moveit_msgs/ExecuteTrajectoryAction.h"

#include "herkulex_interface.h"

void Herkulex::Servo::reboot()
{
	std::vector<u_char> packet = interface->makeCommandPacket(id, REBOOT, std::vector<u_char>(0));
	boost::asio::write(*(interface->port), boost::asio::buffer(packet));
}

Herkulex::Interface::Interface(std::string portName, int BAUD)
{
	try{
		port = new boost::asio::serial_port(io, portName);
		port->set_option(boost::asio::serial_port_base::baud_rate(BAUD));
	}
	catch (boost::system::system_error e)
	{
		connectionError = "Failed to open port \"" + portName + "\" : " + std::string(e.what());
		connected = false;
		return;
	}

	detectServos();
	if (servos.size() == 0)
	{
		connectionError = "Error : No herculex servos detected!";
		connected = false;
		port->close();
		return;
	}

	// reboot all servos and wait for them to come up again
	for (int s=0; s<servos.size(); ++s)
		servos[s].reboot();
	ros::Duration(1.0).sleep();

	// default shutdown policy
	shutdownPolicy = Slack;

	connectionError = "Okay";
	connected = true;
}

Herkulex::Interface::~Interface()
{
	if (connected)
	{
		// Apply joint shutdown policy
		u_char torqueMode;
		if (shutdownPolicy == Brake)
			torqueMode = TORQUE_CONTROL_BREAK_ON;
		else
			torqueMode = TORQUE_CONTROL_TORQUE_FREE;

		for (int s=0; s<servos.size(); ++s)
		{
			setTorqueMode(servos[s].id, torqueMode);
			setLed(servos[s].id, LED_CONTROL_RED);
		}

		// close serial port
		port->close();
	}
}

std::string Herkulex::Interface::getErrorString(u_char statusError, u_char statusDetail)
{
	if (statusError == 0)
		return "okay";
	else
	{
		std::string status = "";

		if (statusError & STATUS_ERROR_VOLTAGE_EXCEEDED)
			status += "[Voltage Exceeded]";
		if (statusError & STATUS_ERROR_RANGE_EXCEEDED)
			status += "[Range Exceeded]";
		if (statusError & STATUS_ERROR_TEMPERATURE_EXCEEDED)
			status += "[Temp Exceeded]";
		if (statusError & STATUS_ERROR_INVALID_PACKET)
		{
			status += "[Invalid Packet -> ";
			if (statusDetail & STATUS_DETAIL_CHECKSUM_ERROR)
				status += "Checksum Error]";
			if (statusDetail & STATUS_DETAIL_UNKNOWN_COMMAND)
				status += "Unknown Command]";
			if (statusDetail & STATUS_DETAIL_EXCEEDED_REG_RANGE)
				status += "Exceeded Register Range]";
			if (statusDetail & STATUS_DETAIL_GARBAGE_DETECTED)
				status += "Garbage Detected]";

		}
		if (statusError & STATUS_ERROR_OVERLOAD_DETECTED)
			status += "[Overload]";
		if (statusError & STATUS_ERROR_EEG_REGISTER_DISTORTED)
			status += "[EEG Reg Distorted]";

		return status;
	}
}

std::string Herkulex::Interface::getDetailString(u_char statusDetail)
{
	std::string status = "";

	if (statusDetail & STATUS_DETAIL_MOVING)
		status += "[moving]";
	else
		status += "        ";
	if (statusDetail & STATUS_DETAIL_IN_POSITION)
		status += "[in position]";
	else
		status += "             ";
	if (statusDetail & STATUS_DETAIL_TORQUE_ON)
		status += "[torque on ]";
	else
		status += "[torque off]";

	return status;
}

// Helper functions
std::vector<u_char> Herkulex::Interface::assembleBytes(u_char b1, u_char b2)
{
	std::vector<u_char> vect;
	vect.push_back(b1);
	vect.push_back(b2);
	return vect;
}
std::vector<u_char> Herkulex::Interface::assembleBytes(u_char b1, u_char b2, u_char b3)
{
	std::vector<u_char> vect;
	vect.push_back(b1);
	vect.push_back(b2);
	vect.push_back(b3);
	return vect;
}

// Methods encapsulating the equations to convert radians to and from raw angle values
// This also limits the radians to raw calculation to within the safe range
// This should also do something cleverer in the future where the model of the servo is
// use to automatically determine the equation.
short Herkulex::Interface::radiansToRaw(u_char servoId, float radians)
{
	Herkulex::Servo *servo = getServoById(servoId);

	// limit radians value to safe range for this servo
	if (radians < servo->angleMin)
		radians = servo->angleMin;
	if (radians > servo->angleMax)
		radians = servo->angleMax;

	float angleDegrees = (radians / 3.141592654) * 180;
	angleDegrees += 180;	// change range from 0 <-> 360 to -180 <-> +180  (Fault in data sheet)
	short rawPosition = (angleDegrees / 0.02778) + 9903;
	return rawPosition;
}
float Herkulex::Interface::rawToRadians(u_char servoId, short raw)
{
	float angleDeg = ((float)raw - 9903) * 0.02778;
	angleDeg -= 180;	// change range from 0 <-> 360 to -180 <-> +180  (Fault in data sheet)
	float angleRads = (angleDeg / 180) * 3.141592654;
	return angleRads;
}

std::vector<u_char> Herkulex::Interface::makeCommandPacket(u_char servoId, u_char command, std::vector<u_char> data)
{
	u_char packetSize = 7 + data.size();
	u_char checksum1 = packetSize ^ servoId ^ command;

	for (int i = 0; i < data.size(); i++)
		checksum1 ^= data[i];

	checksum1 &= 0xFE;
	u_char checksum2 = (~checksum1) & 0xFE;

	std::vector<u_char> commandPacket(2, 0xFF);
	commandPacket.push_back(packetSize);
	commandPacket.push_back(servoId);
	commandPacket.push_back(command);
	commandPacket.push_back(checksum1);
	commandPacket.push_back(checksum2);

	commandPacket.insert(commandPacket.end(), data.begin(), data.end());

	return commandPacket;
}

/** Method to read an acknowledgement response from the port.
 *
 * This is quite basic for now. But will need to be extended
 * to verify the header, checksums and CMD are correct
 */
std::vector<u_char> Herkulex::Interface::getAckResponseFull(u_char cmd, int timeout)
{
	// read return packet using blocking timeout reads
	blockingReader reader(*port, timeout);

	std::vector<u_char> response;
	int bytesToRead = 3;
	bool timedout = false;
	for (int b=0; b<bytesToRead; ++b)
	{
	   char byte;
	   if (!reader.readChar(byte))
	   {
		   timedout = true;
		   break;
	   }
	   response.push_back(byte);

	   // if this was the third byte, it is the length so update the bytes to read
	   if (b == 2)
		   bytesToRead = byte;
   }

   if (timedout)
   {
	   return std::vector<u_char>(0);
   }

   return response;
}

std::vector<u_char> Herkulex::Interface::getAckResponse(u_char cmd, int timeout)
{
	std::vector<u_char> response = getAckResponseFull(cmd, timeout);

	if (response.size() <= 7)
		return std::vector<u_char>(0);

	// split out the actual response (including error and detail status bytes)
	std::vector<u_char> returnedData;
	returnedData.insert(returnedData.begin(), response.begin()+7, response.end());
	return returnedData;
}

std::vector<u_char> Herkulex::Interface::ramRead(u_char servoId, u_char addr, u_char length)
{
	std::vector<u_char> data = assembleBytes(addr, length);
	std::vector<u_char> packet = makeCommandPacket(servoId, RAM_READ, data);

	boost::asio::write(*port, boost::asio::buffer(packet));
	std::vector<u_char> response = getAckResponse(RAM_READ);

	// if valid response data was not returned.
	if (response.size() < 3)
		return std::vector<u_char>(0);

	// split out the actual returned data
   std::vector<u_char> returnedData;
   returnedData.insert(returnedData.begin(), response.begin()+2, response.end());
   return returnedData;
}

void Herkulex::Interface::ramWrite(u_char servoId, std::vector<u_char> data)
{
   std::vector<u_char> packet = makeCommandPacket(servoId, RAM_WRITE, data);
   boost::asio::write(*port, boost::asio::buffer(packet));
}

/// Method to read EEP registers. returns len bytes starting at addr
std::vector<u_char> Herkulex::Interface::eepReadFull(u_char servo_id, u_char addr, u_char len, int timeout)
{
	std::vector<u_char> data = assembleBytes(addr, len);
	std::vector<u_char> packet = makeCommandPacket(servo_id, EEP_READ, data);

	boost::asio::write(*port, boost::asio::buffer(packet));
	return getAckResponseFull(EEP_READ, timeout);
}

/// Method to read EEP registers. returns len bytes starting at addr
std::vector<u_char> Herkulex::Interface::eepRead(u_char servo_id, u_char addr, u_char len, int timeout)
{
	std::vector<u_char> data = assembleBytes(addr, len);
	std::vector<u_char> packet = makeCommandPacket(servo_id, EEP_READ, data);

	boost::asio::write(*port, boost::asio::buffer(packet));
	return getAckResponse(EEP_READ, timeout);
}

void Herkulex::Interface::setLed(u_char servoId, u_char ledState)
{
   std::vector<u_char> data = assembleBytes(LED_CONTROL_RAM_ADDR, 1, ledState);
   ramWrite(servoId, data);
}

void Herkulex::Interface::setTorqueMode(u_char servoId, u_char torqueMode)
{
   std::vector<u_char> data = assembleBytes(TORQUE_CONTROL_RAM_ADDR, 1, torqueMode);
   ramWrite(servoId, data);
}

/** Method to detect all servos connected to the string
 *
 * Returns a std::vector of structures containing the
 * servoId, type and firmware version string
 */
void Herkulex::Interface::detectServos()
{
	// attempt to read model and version info for every servo from 0 - 253
	// 254 is the broadcast addr and 255 is reserved
	for (int s=0; s<=253; ++s)
	{
		std::vector<u_char> data = eepReadFull(s, 0, 4, 1);

		//ROS_WARN("eepRead returned %d bytes", (int)data.size());

		if (data.size() >= 13)
		{
			char typeString[20];
			sprintf(typeString, "DSR-%d%d%d%d",
					data[9] >> 4,
					data[9] & 0xF,
					data[10] >> 4,
					data[10] & 0xF);

			char versionString[30];
			sprintf(versionString, "version %d.%d",
					data[11],
					data[12]);

			Servo servo(data[3], std::string(typeString), std::string(versionString), this);
			servos.push_back(servo);
		}
	}

	// some responses may still be in the serial buffer so try and
	// read them for 0.1 seconds more.
	ros::Time start = ros::Time::now();
	ros::Duration listenTime = ros::Duration(0.1);
	while (ros::Time::now() < start + listenTime)
	{
		std::vector<u_char> data = getAckResponseFull(EEP_READ, 10);

		if (data.size() > 13)
		{
			char typeString[20];
			sprintf(typeString, "DSR-%d%d%d%d",
					data[9] >> 4,
					data[9] & 0xF,
					data[10] >> 4,
					data[10] & 0xF);

			char versionString[30];
			sprintf(versionString, "version %d.%d",
					data[11],
					data[12]);

			Servo servo(data[3], std::string(typeString), std::string(versionString), this);
			servos.push_back(servo);
		}
	}
}

/*struct ServoPowerStatus
{
	float voltageVolts;
	int tempDegrees;
};*/

Herkulex::ServoPowerStatus Herkulex::Interface::getServoPowerStatus(u_char servoId)
{
	Herkulex::ServoPowerStatus status;

	std::vector<u_char> data = ramRead(servoId, 54, 2);

	if (data.size() == 0)
	{
		status.voltageVolts = std::numeric_limits<float>::quiet_NaN();
		status.tempDegrees = -1;
	}
	else
	{
		status.voltageVolts = data[0] / 10.0;

		// decode binary coded decimal of temperature.
		status.tempDegrees = (data[1] & 0x0f) + 10 * (data[1] >> 4);
	}

	return status;
}

std::vector<Herkulex::ServoPowerStatus> Herkulex::Interface::getServoPowerStatuses()
{
	std::vector<Herkulex::ServoPowerStatus> statuses;

	for (int s=0; s<servos.size(); ++s)
		statuses.push_back(getServoPowerStatus(servos[s].id));

	return statuses;
}

Herkulex::ServoJointStatus Herkulex::Interface::getJointState(u_char servoId)
{
	ServoJointStatus status;

	// get servo joint information
	std::vector<u_char> data = ramRead(servoId, 58, 10);

	if (data.size() == 12)
		status.known = true;
	else
	{
		status.known = false;
		return status;
	}

	status.error = data[10];
	status.detail = data[11];

	unsigned int calibrationPositionRaw = 0;
	calibrationPositionRaw |= data[0];
	calibrationPositionRaw |= ((unsigned int)data[1] << 8);
	float calibrationAngleDeg = ((float)calibrationPositionRaw - 9903) * 0.02778;

	// extract joint position
	short positionRaw = 0;
	positionRaw |= data[2];
	positionRaw |= ((unsigned int)data[3] << 8);
	status.angleRads = rawToRadians(servoId, positionRaw);

	// extract joint speed
	short speedRaw = 0;
	speedRaw |= data[4];
	speedRaw |= ((short)data[5] << 8);
	float speedDegPerSec = speedRaw * 0.62;
	status.velocityRadsPerSec = (speedDegPerSec / 180) * 3.141592654;

	// extract joint torque (units unknown and observed values don't make much sense!)
	unsigned int torqueRaw = 0;
	torqueRaw |= data[8];
	torqueRaw |= ((unsigned int)data[9] << 8);
	status.torqueNm = torqueRaw;

	return status;
}

std::vector<Herkulex::ServoJointStatus> Herkulex::Interface::getJointStates()
{
	std::vector<Herkulex::ServoJointStatus> jointStates;

	for (int s=0; s<servos.size(); ++s)
		jointStates.push_back(getJointState(servos[s].id));

	return jointStates;
}

// create an S_JOG packet from the given servoId, time (seconds), angle(degrees) and led status mask
std::vector<u_char> Herkulex::Interface::makeSJOGPacket(Herkulex::TrajectoryPoint position, u_char ledStatus)
{
	std::vector<u_char> packet;

	unsigned int timeOffset = position.timeFromStartSecs / 0.0112;
	if (timeOffset >= 256)
		printf("Warning: time beyond %f second limit!\n", 255 * 0.0112); fflush(stdout);
	packet.push_back(timeOffset & 0xff);

	short rawPosition = radiansToRaw(position.servoId, position.angle);
	packet.push_back(rawPosition & 0xff);
	packet.push_back((rawPosition >> 8) & 0xff);

	// create an options byte for a position jog command with VOR
	u_char options = 0x00 | ((ledStatus & 0x07) << 2);
	packet.push_back(options);

	packet.push_back(position.servoId);

	return packet;
}

void Herkulex::Interface::S_JOGCommand(u_char servoId, std::vector<u_char> packets)
{
	std::vector<u_char> packet = makeCommandPacket(servoId, S_JOG, packets);

	boost::asio::write(*port, boost::asio::buffer(packet));
}

Herkulex::Servo *Herkulex::Interface::getServoById(u_char servoId)
{
	for (int s=0; s<servos.size(); ++s)
		if (servos[s].id == servoId)
			return &(servos[s]);

	return NULL;
}

bool Herkulex::Interface::jogServo(Herkulex::TrajectoryPoint position)
{
	std::vector<u_char> jogPacket = makeSJOGPacket(position, LED_CONTROL_BLUE);
	S_JOGCommand(position.servoId, jogPacket);

	return true;
}


