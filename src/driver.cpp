
#include <math.h>
#include <stdlib.h>
#include <boost/algorithm/string.hpp>
#include <mutex>

// ROS libraries
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/JointState.h>
#include <actionlib/server/simple_action_server.h>

// Standard messages/actions
#include "moveit_msgs/ExecuteTrajectoryAction.h"

 // Custom messages
#include "ros_herkulex/JointState.h"
#include "ros_herkulex/Jog.h"
#include "ros_herkulex/JogMultiple.h"

#include "herkulex_interface.h"

class HerkulexDriver
{
public:
	HerkulexDriver();
	~HerkulexDriver();

	void positionCallback(std_msgs::Float32 msg);

private:

	sensor_msgs::JointState makeJointStateMsg(std::vector<std::string> jointNames,
											  std::vector<Herkulex::ServoJointStatus> joints);

	ros_herkulex::JointState makeHerkulexJointStateMsg(std::vector<std::string> jointNames,
													   std::vector<Herkulex::ServoJointStatus> joints,
													   std::vector<Herkulex::ServoPowerStatus> jointStatuses);

	void publishJointStatusMessages();

	void runTrajectory(Herkulex::TrajectoryPoint goal);

	void jogCallback(ros_herkulex::Jog msg);

	void jogMultipleCallback(ros_herkulex::JogMultiple msg);

	std::vector<float> interpolateTrajectoryJointAngles(const trajectory_msgs::JointTrajectory trajectory,
														ros::Duration timeOnTraj);

	void trajectoryHandler(const moveit_msgs::ExecuteTrajectoryGoalConstPtr &goal);

	ros::NodeHandle n;
	actionlib::SimpleActionServer<moveit_msgs::ExecuteTrajectoryAction> trajectoryActionServer;

	Herkulex::Interface *interface;
	std::mutex interfaceLock;

	std::vector<std::string> jointNames;
	std::vector<Herkulex::ServoJointStatus> jointStates;

	ros::Publisher jointPublisher;
	ros::Publisher herkulexJointPublisher;

	int jointPublisherRate;

	// queue of jog commands queued up in the future
	// these will be executed when they become due in the main loop of the driver
	// any new jog or trajectory commands will clear the future queue.
	std::vector<ros::Time> queuedTimes;
	std::vector<Herkulex::TrajectoryPoint> queuedCommands;

	float jointInitialAngleToleranceRads;
	float jointGoalAngleToleranceRads;

	// Lookup table to translate from joint indices in the current trajectory action and to servo indices.
	std::vector<int> trajectoryJointLookup;
};

HerkulexDriver::HerkulexDriver() : trajectoryActionServer(n, "/trajectory_control", boost::bind(&HerkulexDriver::trajectoryHandler, this, _1), false)
{
	ros::NodeHandle n("~");

	// get and validate parameters
	std::string portName, jointNamesParam, startupPolicy, shutdownPolicy;
	int BAUDRate;
		n.param<std::string>("port", portName, "/dev/ttyUSB0");
		n.param<int>("baud", BAUDRate, 115200);
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

	jointPublisher = n.advertise<sensor_msgs::JointState>("/joint_states", 10);
	herkulexJointPublisher = n.advertise<ros_herkulex::JointState>("/herkulex_joint_states", 10);

	ros::Subscriber sub = n.subscribe("/position", 10, &HerkulexDriver::positionCallback, this);
	ros::Subscriber jogSubscriber = n.subscribe("/jog", 10, &HerkulexDriver::jogCallback, this);
	ros::Subscriber jogMultipleSubscriber = n.subscribe("/jog_multiple", 10, &HerkulexDriver::jogMultipleCallback, this);

	ROS_INFO("connecting to Herkulex servo string on port \"%s\" with BAUD %d", portName.c_str(), BAUDRate);
	interface = new Herkulex::Interface(portName, BAUDRate);

	printf("created hardware interface.\n"); fflush(stdout);

	if (!interface->isConnected())
	{
		ROS_ERROR("Error : Failed to setup Herkulex interface [%s]", interface->getConnectionError().c_str());
		exit(1);
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

	jointInitialAngleToleranceRads = 0.01151917; // 0.66 degrees in radians.
	jointGoalAngleToleranceRads = 0.01151917;

	trajectoryActionServer.start();
	ROS_INFO("Started trajectory action server.");

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
}

HerkulexDriver::~HerkulexDriver()
{
    delete interface;
}

std::vector<float> HerkulexDriver::interpolateTrajectoryJointAngles(const trajectory_msgs::JointTrajectory trajectory,
																	ros::Duration timeOnTraj)
{
	std::vector<float> jointAngles(trajectory.joint_names.size(), 0.0);

	// check time is within trajectory period
	if (timeOnTraj.toSec() < 0)
	{
		ROS_ERROR("Error : Trying to interpolate a point on a trajectory with a negative time!");
		return jointAngles;
	}
	int trajectoryPointCount = trajectory.points.size();
	ros::Duration trajectoryDuration = trajectory.points[trajectoryPointCount-1].time_from_start;
	if (timeOnTraj > trajectoryDuration)
	{
		ROS_ERROR("Error : Trying to interpolate a point on a trajectory with a time beyond trajectory duration!");
		return jointAngles;
	}

	//printf("Duration into trajectory okay.\n"); fflush(stdout);
	//printf("Interpolating %f secs of a %f second trajectory.\n", timeOnTraj.toSec(), trajectoryDuration.toSec()); fflush(stdout);

	// get two adjacent points and time ratio
	int p;
	float ratio;
	for (p=0; p<trajectoryPointCount-1; p++)
		if (timeOnTraj >= trajectory.points[p  ].time_from_start &&
			timeOnTraj <  trajectory.points[p+1].time_from_start)
		{
			float segmentDuration = (trajectory.points[p+1].time_from_start - trajectory.points[p].time_from_start).toSec();
			float segmentPosition = (timeOnTraj - trajectory.points[p].time_from_start).toSec();
			ratio = segmentPosition / segmentDuration;
			break;
		}

	//printf("Two adjacent indices are %d - %d out of %d points.\n", p, p+1, trajectoryPointCount); fflush(stdout);

	// interpolate final joint angles
	for (int j=0; j<jointAngles.size(); ++j)
	{
		jointAngles[j] = trajectory.points[p].positions[j] * (1-ratio) + trajectory.points[p+1].positions[j] * ratio;
	}

	return jointAngles;
}

void HerkulexDriver::trajectoryHandler(const moveit_msgs::ExecuteTrajectoryGoalConstPtr &goal)
{
	// verify there is at least one joint in the trajectory
	if (goal->trajectory.joint_trajectory.joint_names.size() == 0)
	{
		moveit_msgs::ExecuteTrajectoryResult result;
		result.error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN;
		trajectoryActionServer.setAborted(result, "Error : no joints defined in joint_trajectory!");
		return;
	}

	// verify that the joint names listed in the trajectory exist and make joint lookup table
	trajectoryJointLookup.clear();
	for (int j=0; j<goal->trajectory.joint_trajectory.joint_names.size(); ++j)
	{
		auto pos = std::find(jointNames.begin(), jointNames.end(), goal->trajectory.joint_trajectory.joint_names[j]);
		if (pos == jointNames.end())
		{
			moveit_msgs::ExecuteTrajectoryResult result;
			result.error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_LINK_NAME;
			std::string errorMsg = "Error : joint name \"" + goal->trajectory.joint_trajectory.joint_names[j] + "\" doesn't exist in robot!";
			trajectoryActionServer.setAborted(result, errorMsg);
			return;
		}

		trajectoryJointLookup.push_back(pos - jointNames.begin());

		ROS_INFO("JointLookup [%d] -> %d", j, (int)(pos - jointNames.begin()));
	}

	interfaceLock.lock();
	std::vector<Herkulex::ServoJointStatus> currentJointStates = interface->getJointStates();
	interfaceLock.unlock();

	for (int j=0; j<goal->trajectory.joint_trajectory.joint_names.size(); ++j)
	{
		Herkulex::ServoJointStatus jointStatus = currentJointStates[trajectoryJointLookup[j]];
		printf("Got a position of %f rads for servo id %d",
				jointStatus.angleRads,
				jointStatus.servoId);

		double trajectoryStartAngle = goal->trajectory.joint_trajectory.points[0].positions[j];
		float servoStartAngle = jointStatus.angleRads;

		if (fabs(trajectoryStartAngle - servoStartAngle) >= jointInitialAngleToleranceRads)
		{
			ROS_WARN("Joint [%s] start state error of %f radians.",
					 goal->trajectory.joint_trajectory.joint_names[j].c_str(),
					 (trajectoryStartAngle - servoStartAngle));

			/*moveit_msgs::ExecuteTrajectoryResult result;
			result.error_code.val = moveit_msgs::MoveItErrorCodes::START_STATE_VIOLATES_PATH_CONSTRAINTS;
			std::string errorMsg = "Error : joint " + goal->trajectory.joint_trajectory.joint_names[j] + " violates initial state tolerance!";
			trajectoryActionServer.setAborted(result, errorMsg);
			return;*/
		}
	}

	// run fake simulation of trajectory for now
	ros::Time start = ros::Time::now();
	int trajPointsCount = goal->trajectory.joint_trajectory.points.size();
	ros::Duration trajDuration = goal->trajectory.joint_trajectory.points[trajPointsCount-1].time_from_start;

	printf("Calculated a trajectory duration of %f seconds.\n", trajDuration.toSec());

	float overshootRatio = 1.3;

	ros::Rate loopRate(jointPublisherRate);
	while (ros::Time::now() < start + trajDuration)
	{

		// interpolate joint angles at this point of time
		std::vector<float> jointAngles = interpolateTrajectoryJointAngles(goal->trajectory.joint_trajectory,
																		  ros::Time::now() - start);

		printf("made joint angles.\n"); fflush(stdout);

		// add overshoot to joint angles
		for (int j=0; j<jointAngles.size(); ++j)
		{
			float currentAngle = currentJointStates[trajectoryJointLookup[j]].angleRads;
			jointAngles[j] = currentAngle + ((jointAngles[j] - currentAngle) * overshootRatio);
		}

		// send multiple jog command to servo interface
		// create vector of trajectory point objects and send to the interface
		std::vector<Herkulex::TrajectoryPoint> jogs;
		for (int j=0; j<jointAngles.size(); ++j)
		{
			printf("Adding jog: id[%d] time[%f sec] angle[%f rads]\n",
				   jointStates[trajectoryJointLookup[j]].servoId,
				   loopRate.expectedCycleTime().toSec(),
				   jointAngles[j]); fflush(stdout);

			jogs.push_back(Herkulex::TrajectoryPoint(currentJointStates[trajectoryJointLookup[j]].servoId,
													 loopRate.expectedCycleTime().toSec() * overshootRatio,
													 jointAngles[j]));
		}

		printf("made trajctory point msgs %d.\n", (int)jogs.size()); fflush(stdout);

		interfaceLock.lock();
		interface->jogServos(jogs);
		interfaceLock.unlock();

		printf("sent jogs to interface.\n"); fflush(stdout);

		// add fake joint angles into jointStates vector
		//for (int j=0; j<goal->trajectory.joint_trajectory.joint_names.size(); ++j)
		//	jointStates[trajectoryJointLookup[j]].angleRads = jointAngles[j];

		// publish fake joint states
		//sensor_msgs::JointState fakeJointStateMsg = makeJointStateMsg(jointNames, jointStates);
		//jointPublisher.publish(fakeJointStateMsg);

		moveit_msgs::ExecuteTrajectoryFeedback progressMsg;
		progressMsg.state = "Trajectory in Progress ";
		trajectoryActionServer.publishFeedback(progressMsg);

		loopRate.sleep();
	}

	moveit_msgs::ExecuteTrajectoryResult result;
	result.error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
	trajectoryActionServer.setSucceeded(result, "Trajectory completed okay.");

  // copy the robot trajectory to a property of the driver object and set the status to following trajectory
  //activeTrajectory = goal->trajectory.joint_trajectory.points;
  //trajectoryStartTime = ros::Time::now();
  //armStatus = ARM_FOLLOWING_TRAJECTORY;
}

sensor_msgs::JointState HerkulexDriver::makeJointStateMsg(std::vector<std::string> jointNames,
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

ros_herkulex::JointState HerkulexDriver::makeHerkulexJointStateMsg(std::vector<std::string> jointNames,
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


void HerkulexDriver::publishJointStatusMessages()
{
	bool gotJointStates = false;

	// produce and publish joint state msg if any nodes are listening
	if (jointPublisher.getNumSubscribers() > 0)
	{
		interfaceLock.lock();
		jointStates = interface->getJointStates();
		interfaceLock.unlock();
		gotJointStates = true;

		sensor_msgs::JointState jointStateMsg = makeJointStateMsg(jointNames, jointStates);
		jointPublisher.publish(jointStateMsg);
	}

	// produce and publish herkulex joint state msg if any nodes are listening
	if (herkulexJointPublisher.getNumSubscribers() > 0)
	{
		interfaceLock.lock();

		if (!gotJointStates)
		{
			jointStates = interface->getJointStates();
    		gotJointStates = true;
		}

		std::vector<Herkulex::ServoPowerStatus> jointStatuses = interface->getServoPowerStatuses();
		interfaceLock.unlock();

		ros_herkulex::JointState herkulexJointStateMsg = makeHerkulexJointStateMsg(jointNames,
																				   jointStates,
																				   jointStatuses);
		herkulexJointPublisher.publish(herkulexJointStateMsg);
	}
}

void HerkulexDriver::positionCallback(std_msgs::Float32 msg)
{
	float value = msg.data;

	Herkulex::TrajectoryPoint test(219, 1.0, 3.141);
	test.setDegrees(value);
	interfaceLock.lock();
	interface->jogServo(test);
	interfaceLock.unlock();
}

void HerkulexDriver::runTrajectory(Herkulex::TrajectoryPoint goal)
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
	interfaceLock.lock();
	interface->jogServo(points[2]);
	interfaceLock.unlock();

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

void HerkulexDriver::jogCallback(ros_herkulex::Jog msg)
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
	{
		interfaceLock.lock();
		interface->jogServo(jog);
		interfaceLock.unlock();
	}
}

void HerkulexDriver::jogMultipleCallback(ros_herkulex::JogMultiple msg)
{
	// clear any queued commands
	queuedTimes.clear();
	queuedCommands.clear();

	ROS_WARN("Received a multiple jog messages for %d servos.", (int)msg.joint_jogs.size());

	// verify joint names and get servo ids
	std::vector<int> servoIds;
	for (int j=0; j<msg.joint_jogs.size(); ++j)
	{
		int servoId = -1;
		for (int n=0; n<jointNames.size(); ++n)
			if (jointNames[n] == msg.joint_jogs[j].joint_name)
				servoId = interface->servos[n].id;

		if (servoId == -1)
		{
			ROS_ERROR("Error : jog multiple failed, joint name \"%s\" doesn't exist.", msg.joint_jogs[j].joint_name.c_str());
			return;
		}
		servoIds.push_back(servoId);

		ROS_INFO("Joint \"%s\" found servo id %d", msg.joint_jogs[j].joint_name.c_str(), servoId);
	}

	// for each jog message if a goal speed was given then use it to calculate the time it will take
	// (herkulex commands take position and time not speed directly)
	for (int j=0; j<msg.joint_jogs.size(); ++j)
		if (msg.joint_jogs[j].goal_speed > 0.0)
		{
			Herkulex::ServoJointStatus state = interface->getJointState(servoIds[j]);

			// find time needed to reach goal angle at given rate
			msg.joint_jogs[j].goal_time = fabs(msg.joint_jogs[j].goal_angle - state.angleRads) / msg.joint_jogs[j].goal_speed;
		}

	// create vector of trajectory point objects and send to the interface
	std::vector<Herkulex::TrajectoryPoint> jogs;
	for (int j=0; j<msg.joint_jogs.size(); ++j)
		jogs.push_back(Herkulex::TrajectoryPoint(servoIds[j],
												 msg.joint_jogs[j].goal_time,
												 msg.joint_jogs[j].goal_angle));

	interfaceLock.lock();
	interface->jogServos(jogs);
	interfaceLock.unlock();

	// if the time is beyond the protocol limit of 2.856000 seconds then run this
	// as a trajectory instead.
	//if (time >= 2.856)
	//	runTrajectory(jog);
	//else
	//	interface->jogServo(jog);
}

int main(int argc, char **argv)
{
	ROS_INFO("--[ Herkulex Driver Node ]--");
	ros::init(argc, argv, "herkulex_driver");
	ros::start();

	HerkulexDriver driver;

	ROS_INFO("--[ Herkulex Driver Node: shutdown OK ]--\n");

	return 0;
}
