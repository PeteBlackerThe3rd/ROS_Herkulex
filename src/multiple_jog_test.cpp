
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
#include "ros_herkulex/JogMultiple.h"

#include "herkulex_interface.h"

std::vector<std::string> jointNames;

void jointStateCallback(const sensor_msgs::JointStateConstPtr &jointStates)
{
	jointNames = jointStates->name;
	ROS_INFO("Received joint state msg.");
}

int main(int argc, char **argv)
{
	ROS_INFO("--[ multiple jog test node ]--");
	ros::init(argc, argv, "multiple_jog_test");
	ros::start();

	// setup ros for this node and get handle to ros system

	ros::NodeHandle n("~");

	// get and validate parameters
	double angle, time;
	n.param<double>("angle", angle, 0.0);
	n.param<double>("time", time, 2.0);

	ros::Publisher jogPublisher = n.advertise<ros_herkulex::JogMultiple>("/jog_multiple", 10);

	ros::Subscriber jointStates = n.subscribe("/joint_states", 10, &jointStateCallback);
	//ros::Subscriber jogSubscriber = n.subscribe("/jog", 10, &HerkulexDriver::jogCallback, this);

	// allow time for publisher to connect
	ros::Duration(2.0).sleep();

	// waiting for joint state callback
	ros::Rate loopRate(20);
	ROS_INFO("Waiting for joint_states topic message.");
	while(ros::ok() && jointNames.size() == 0)
	{
    	ros::spinOnce();
    	loopRate.sleep();
	}
	ROS_INFO("Done.");

	// make multiple jog message
	ros_herkulex::JogMultiple msg;
	for (int j=0; j<jointNames.size(); ++j)
	{
		ros_herkulex::Jog jogMsg;
		jogMsg.joint_name = jointNames[j];
		jogMsg.goal_angle = angle;
		jogMsg.goal_time = time;
		jogMsg.goal_speed = 0.0;
		msg.joint_jogs.push_back(jogMsg);
	}

	jogPublisher.publish(msg);

	ROS_INFO("Published jog multiple message.");

	return 0;
}
