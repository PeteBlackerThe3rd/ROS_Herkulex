
#include <math.h>
#include <stdlib.h>

// ROS libraries
#include <ros/ros.h>

#include "herkulex_interface.h"

Herkulex::Interface *interface;

int main(int argc, char **argv)
{
	ROS_INFO("--[ Herkulex Servo Setup Node ]--");

	// setup ros for this node and get handle to ros system
	ros::init(argc, argv, "herkulex_driver");
	ros::start();

	ros::NodeHandle n("~");

	// get and validate parameters
	std::string portName;
	int BAUDRate;
	n.param<std::string>("herkulex_port", portName, "/dev/ttyUSB0");
	n.param<int>("herkulex_baud", BAUDRate, 115200);

	ROS_INFO("connecting to Herkulex servo string on port \"%s\" with BAUD %d", portName.c_str(), BAUDRate);
	interface = new Herkulex::Interface(portName, BAUDRate);

	if (!interface->isConnected())
	{
		ROS_ERROR("Error : Failed to setup Herkulex interface [%s]", interface->getConnectionError().c_str());
		return 1;
	}

	// write detected servo info to log
	std::vector<Herkulex::Servo> servos = interface->getServos();
	ROS_INFO("Detected %d herculex servos :", (int)servos.size());
	for (int s=0; s<servos.size(); ++s)
		ROS_INFO("%s", servos[s].toString().c_str());

	// set all servo LEDs to green
	for (int s=0; s<servos.size(); ++s)
	{
		interface->setLed(servos[s].id, LED_CONTROL_GREEN);
	}

	std::string command;
	n.param<std::string>("cmd", command, "no command");

	// check if set servo id command was sent.
	if (command == "set_id")
	{
		int oldServoId, newServoId;
		n.param<int>("id", oldServoId, -1);
		n.param<int>("new_id", newServoId, -1);

		if (oldServoId == -1 || newServoId == -1)
			ROS_ERROR("Error : Cannot set servo id, old and new id's not given.");
		else if (oldServoId < 0 || oldServoId >= 0xFE)
			ROS_ERROR("Error : Invalid id, must be between 0 - 253");
		else if (newServoId < 0 || newServoId >= 0xFE)
			ROS_ERROR("Error : Invalid new_id, must be between 0 - 253");
		else if (newServoId == oldServoId)
			ROS_ERROR("Error : old and new servo ids are identical, nothing to do!");
		else
		{
			// commands set okay, now check that the old servo id exists and that the new id doesn't conflict
			bool idExists = false;
			bool newIdExists = false;
			for (int s=0; s<interface->servos.size(); ++s)
			{
				if (interface->servos[s].id == oldServoId)
					idExists = true;
				if (interface->servos[s].id == newServoId)
					newIdExists = true;
			}

			if (!idExists)
				ROS_ERROR("Error : Cannot set servo id, old servo id of [%d] doesn't exist in chain.", oldServoId);
			else if (newIdExists)
				ROS_ERROR("Error : Cannot set servo id, new servo id of [%d] conflicts with an existing servo in the chain.", newServoId);
			else
			{
				// Everything has now been validated, so update id
				ROS_INFO("Updating ID of servo [%d] to [%d]", oldServoId, newServoId);
				interface->setServoId(oldServoId, newServoId);

				// reload interface and display new list of servos
				delete interface;
				interface = new Herkulex::Interface(portName, BAUDRate);

				// write new detected servo info to log
				servos = interface->getServos();
				ROS_INFO("After ID update, Detected %d herculex servos :", (int)servos.size());
				for (int s=0; s<servos.size(); ++s)
					ROS_INFO("%s", servos[s].toString().c_str());

				ROS_INFO("---------------------------------------------------");
			}
		}
	}

	// check if identifiy servo command was sent
	else if (command == "identify")
	{
		int servoId;
		n.param<int>("id", servoId, -1);

		if (servoId == -1)
			ROS_ERROR("Error : Cannot identify servo id not given.");
		else if (servoId < 0 || servoId >= 0xFE)
			ROS_ERROR("Error : Invalid id, must be between 0 - 253");
		else
		{
			// commands set okay, now check that the old servo id exists and that the new id doesn't conflict
			bool idExists = false;
			Herkulex::Servo *servo;
			for (int s=0; s<interface->servos.size(); ++s)
				if (interface->servos[s].id == servoId)
				{
					idExists = true;
					servo = &interface->servos[s];
				}

			if (!idExists)
				ROS_ERROR("Error : Cannot identify servo. Id of [%d] doesn't exist in chain.", servoId);
			else
			{
				// now loop until node shutdown flashing the specified servo's LED through red, green and blue
				ROS_INFO("Flashing servo with Id [%d] : type : %s", servoId, servo->type.c_str());

				ros::Rate rate(10);
				int count = 0;
				while(ros::ok())
				{
					switch(count)
					{
					case 0: interface->setLed((u_char)servoId, LED_CONTROL_RED); count=1; break;
					case 1: interface->setLed((u_char)servoId, LED_CONTROL_GREEN); count=2; break;
					case 2: interface->setLed((u_char)servoId, LED_CONTROL_BLUE); count=0; break;
					}

					rate.sleep();
				}
			}
		}
	}

	// check if identifiy all servos command was sent
	else if (command == "identify_all")
	{
		if (servos.size() == 0)
		{
			ROS_ERROR("Error : No servos connected to identify!");
		}
		else
		{
			// loop through all servos flashing each one for 2 seconds
			int servoIt = 0;
			while(ros::ok())
			{
				ROS_INFO("Identifying Servo Id [%d] type : %s", servos[servoIt].id, servos[servoIt].type.c_str());

				ros::Rate rate(10);
				int count = 0;
				ros::Duration secondsToFlash = ros::Duration(4.0);
				ros::Time start = ros::Time::now();
				while(ros::ok() && ros::Time::now() < start + secondsToFlash)
				{
					switch(count)
					{
					case 0: interface->setLed(servos[servoIt].id, LED_CONTROL_RED); count=1; break;
					case 1: interface->setLed(servos[servoIt].id, LED_CONTROL_GREEN); count=2; break;
					case 2: interface->setLed(servos[servoIt].id, LED_CONTROL_BLUE); count=0; break;
					}

					rate.sleep();
				}

				// reset servo back to default green color
				interface->setLed(servos[servoIt].id, LED_CONTROL_GREEN);

				ROS_INFO("Done.");

				// move to the next servo in the chain
				servoIt = servoIt+1;
				if (servoIt >= (int)servos.size())
					servoIt = 0;
			}
		}
	}

	else // if no commands were sent display help
	{
		ROS_INFO("---------------------------------------------------");
		ROS_INFO("  No commands given");
		ROS_INFO("  Possible commands:");
		ROS_INFO("  cmd:=set_id id:=<servo id> new_id:=<new servo id>");
		ROS_INFO("  cmd:=identify id:=<servo id>");
		ROS_INFO("  cmd:=identify_all");
		ROS_INFO("---------------------------------------------------");
	}

    delete interface;

	return 0;
}
