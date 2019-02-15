
#include <math.h>
#include <stdlib.h>
#include <boost/algorithm/string.hpp>

// ROS libraries
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/JointState.h>

 // Custom messages
#include "ros_herkulex/JointState.h"
#include "ros_herkulex/Jog.h"

//#include <actionlib/server/simple_action_server.h>
//#include "moveit_msgs/ExecuteTrajectoryAction.h"

#include "herkulex_interface.h"

Herkulex::Interface *interface;
std::vector<std::string> jointNames;

ros::Publisher jointPublisher;
ros::Publisher herkulexJointPublisher;

// queue of jog commands queued up in the future
// these will be executed when they become due in the main loop of the driver
// any new jog or trajectory commands will clear the future queue.
std::vector<ros::Time> queuedTimes;
std::vector<Herkulex::TrajectoryPoint> queuedCommands;


sensor_msgs::JointState makeJointStateMsg(std::vector<std::string> jointNames,
										  std::vector<Herkulex::ServoJointStatus> joints)
{
	static int seqId = 0;

	sensor_msgs::JointState jointStateMsg;

	jointStateMsg.header.seq = seqId++;
	jointStateMsg.header.stamp = ros::Time::now();
	jointStateMsg.header.frame_id = "";

	jointStateMsg.name = jointNames;

	for (int j=0; j<joints.size(); ++j)
	{
		jointStateMsg.position.push_back(joints[j].angleRads);
		jointStateMsg.velocity.push_back(joints[j].velocityRadsPerSec);
	}

	return jointStateMsg;
}

ros_herkulex::JointState makeHerkulexJointStateMsg(std::vector<std::string> jointNames,
												   std::vector<Herkulex::ServoJointStatus> joints,
												   std::vector<Herkulex::ServoPowerStatus> jointStatuses)
{
	static int seqId = 0;

	ros_herkulex::JointState jointStateMsg;

	jointStateMsg.header.seq = seqId++;
	jointStateMsg.header.stamp = ros::Time::now();
	jointStateMsg.header.frame_id = "";

	jointStateMsg.name = jointNames;

	for (int j=0; j<joints.size(); ++j)
	{
		jointStateMsg.position.push_back(joints[j].angleRads);
		jointStateMsg.velocity.push_back(joints[j].velocityRadsPerSec);

		jointStateMsg.servo_id.push_back(interface->servos[j].id);
		jointStateMsg.type.push_back(interface->servos[j].type);
		jointStateMsg.firmware.push_back(interface->servos[j].firmwareVersion);

		jointStateMsg.voltage.push_back(jointStatuses[j].voltageVolts);
		jointStateMsg.temperature.push_back(jointStatuses[j].tempDegrees);

		short status = (short)joints[j].error | ((short)joints[j].detail << 8);
		jointStateMsg.status.push_back(status);
	}

	return jointStateMsg;
}


void publishJointStatusMessages()
{
	bool gotJointStates = false;
	std::vector<Herkulex::ServoJointStatus> joints;

	// produce and publish joint state msg if any nodes are listening
	if (jointPublisher.getNumSubscribers() > 0)
	{
		joints = interface->getJointStates();
		gotJointStates = true;

		sensor_msgs::JointState jointStateMsg = makeJointStateMsg(jointNames, joints);
		jointPublisher.publish(jointStateMsg);
	}

	// produce and publish herkulex joint state msg if any nodes are listening
	if (herkulexJointPublisher.getNumSubscribers() > 0)
	{
		if (!gotJointStates)
		{
    		joints = interface->getJointStates();
    		gotJointStates = true;
		}

		std::vector<Herkulex::ServoPowerStatus> jointStatuses = interface->getServoPowerStatuses();

		ros_herkulex::JointState herkulexJointStateMsg = makeHerkulexJointStateMsg(jointNames,
																				   joints,
																				   jointStatuses);
		herkulexJointPublisher.publish(herkulexJointStateMsg);
	}
}

void positionCallback(std_msgs::Float32 msg)
{
	float value = msg.data;

	Herkulex::TrajectoryPoint test(219, 1.0, 3.141);
	test.setDegrees(value);
	interface->jogServo(test);
}

void runTrajectory(Herkulex::TrajectoryPoint goal)
{
	// find the minimum number of segments to split this movement into such that
	// each segment is not longer than half the time limit
	float timeLimit = 2.856;
	int segmentCount = ceil(goal.timeFromStartSecs / (timeLimit / 2));

	ROS_INFO("Split trajectory into %d segments of (%f seconds) for smooth following.", segmentCount,
			goal.timeFromStartSecs / segmentCount);

	// get the current joint angle and create a list of trajectory goals along the way
	Herkulex::ServoJointStatus status = interface->getJointState(goal.servoId);
	std::vector<Herkulex::TrajectoryPoint> points;
	float deltaAngle = goal.angle - status.angleRads;
	for (int s=0; s<=segmentCount; ++s)
	{
		float ratio = s / (float)segmentCount;

		points.push_back(Herkulex::TrajectoryPoint(goal.servoId,
												   (goal.timeFromStartSecs / segmentCount) * 2.0,
												   status.angleRads + (deltaAngle * ratio)));
	}

	//ROS_WARN("Created set of [%d] trajectory point along trajectory", (int)points.size());

	ros::Time startTime = ros::Time::now();
	interface->jogServo(points[2]);

	ros::Duration segmentDuration(goal.timeFromStartSecs / segmentCount);

	// add all other points along the trajectory to the command queue
	for (int s=3; s<=segmentCount; ++s)
	{
		//ROS_WARN("Queueing up trajectory point [%d]", s);

		queuedTimes.push_back(startTime + (segmentDuration * (s-2) ));
		queuedCommands.push_back(points[s]);
	}

	//ROS_WARN("exiting runTrajectory function");
}

void jogCallback(ros_herkulex::Jog msg)
{
	// clear any queued commands
	queuedTimes.clear();
	queuedCommands.clear();

	// verify joint name and get servo id
	int servoId = -1;
	for (int j=0; j<jointNames.size(); ++j)
		if (jointNames[j] == msg.joint_name)
			servoId = interface->servos[j].id;

	if (servoId == -1)
	{
		ROS_ERROR("ERROR : Cannot process herkulex jog message, invalid joint name \"%s\" given.", msg.joint_name.c_str());
		return;
	}

	float time = msg.goal_time;

	// if the goal speed was specified then find the current angle and work out the speed.
	if (msg.goal_speed > 0.0)
	{
		Herkulex::ServoJointStatus state = interface->getJointState(servoId);
		float currentAngle = state.angleRads;

		// find time needed to reach goal angle at given rate
		time = fabs(msg.goal_angle - currentAngle) / msg.goal_speed;
	}

	Herkulex::TrajectoryPoint jog(servoId, time, msg.goal_angle);

	// if the time is beyond the protocol limit of 2.856000 seconds then run this
	// as a trajectory instead.
	if (time >= 2.856)
		runTrajectory(jog);
	else
		interface->jogServo(jog);
}

int main(int argc, char **argv)
{
	ROS_INFO("--[ Herkulex Driver Node ]--");

	// setup ros for this node and get handle to ros system
	ros::init(argc, argv, "herkulex_driver");
	ros::start();

	ros::NodeHandle n("~");

	// get and validate parameters
	std::string portName, jointNamesParam, startupPolicy, shutdownPolicy;
	int BAUDRate, jointPublisherRate;
	n.param<std::string>("herkulex_port", portName, "/dev/ttyUSB0");
	n.param<int>("herkulex_baud", BAUDRate, 115200);
	n.param<int>("joint_publish_rate", jointPublisherRate, 50);
	n.param<std::string>("joint_names", jointNamesParam, "");
	n.param<std::string>("startup_policy", startupPolicy, "torque_on");
	n.param<std::string>("shutdown_policy", shutdownPolicy, "torque_on");

	if (startupPolicy != "torque_on" && startupPolicy != "torque_off")
	{
		ROS_ERROR("Error: Invalid startup_policy parameter \"%s\" must be either \"torque_on\" or \"torque_off\"",
				  startupPolicy.c_str());
		exit(1);
	}
	if (shutdownPolicy != "torque_on" && shutdownPolicy != "torque_off")
	{
		ROS_ERROR("Error: Invalid shutdown_policy parameter \"%s\" must be either \"torque_on\" or \"torque_off\"",
				  shutdownPolicy.c_str());
		exit(1);
	}

	jointPublisher = n.advertise<sensor_msgs::JointState>("joint_states", 10);
	herkulexJointPublisher = n.advertise<ros_herkulex::JointState>("herkulex_joint_states", 10);

	ros::Subscriber sub = n.subscribe("/position", 10, positionCallback);
	ros::Subscriber jogSubscriber = n.subscribe("/jog", 10, jogCallback);

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

	// setup joint names
	if (jointNamesParam == "")
	{
		for (int s=0; s<servos.size(); ++s)
			jointNames.push_back("Joint_" + std::to_string(s+1));
	}
	else
	{
		boost::split(jointNames, jointNamesParam, [](char c){return c == ' ';});

		if (jointNames.size() != servos.size())
		{
			ROS_ERROR("Error: Incorrect number of joint names specified. %d servos and %d names.",
					  (int)servos.size(),
					  (int)jointNames.size());
		}
	}

	ROS_INFO("Joint names set to:");
	for (int j=0; j<jointNames.size(); ++j)
		ROS_INFO("  [%3d] - %s", j, jointNames[j].c_str());

	// apply startup torque policy and set all servo LEDs to green
	for (int s=0; s<servos.size(); ++s)
	{
		interface->setLed(servos[s].id, LED_CONTROL_GREEN);
		if (startupPolicy == "torque_on")
			interface->setTorqueMode(servos[s].id, TORQUE_CONTROL_TORQUE_ON);
		else
			interface->setTorqueMode(servos[s].id, TORQUE_CONTROL_TORQUE_FREE);
	}

	// set shutdown policy
	if (shutdownPolicy == "torque_on")
		interface->setShudownPolicy(Herkulex::Brake);
	else
		interface->setShudownPolicy(Herkulex::Slack);

    ROS_INFO("--[ Herkulex Driver Node: startup complete ]--");

    ros::Rate loopRate(jointPublisherRate);
    int seq=0;
    while(ros::ok())
    {
    	// send joint state messages if any nodes are listening
    	publishJointStatusMessages();

    	//ROS_INFO("Queue size is (%d)", (int)queuedTimes.size());

    	// process queued commands if any are due.
    	if (queuedTimes.size() > 0)
    		if (ros::Time::now() > queuedTimes[0])
    		{
    			interface->jogServo(queuedCommands[0]);

    			queuedTimes.erase(queuedTimes.begin());
    			queuedCommands.erase(queuedCommands.begin());

    			//if (queuedTimes.size() == 0)
    			//	ROS_WARN("All queued commands completed -===========================");
    		}

    	ros::spinOnce();
    	loopRate.sleep();
    }

    delete interface;

	printf("--[ Herkulex Driver Node: shutdown OK ]--\n");
	return 0;
}
