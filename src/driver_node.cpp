/*-----------------------------------------------------------\\
||                                                           ||
||                 FlatBot ROS Driver Node                   ||
||               ----------------------------                ||
||                                                           ||
||                                                           ||
\\-----------------------------------------------------------//

driver_node.cpp

-------------------------------------------------------------*/
#include "driver_node.h"

//#include <time.h>  // used for measuring CPU time of various algorithms
//#include <sys/times.h>

// constructor
FlatBotDriver::FlatBotDriver() :
			trajectoryActionServer(n,
								   "trajectory_control",
								   boost::bind(&FlatBotDriver::trajectoryHandler, this, _1),
								   false)
{
  ros::NodeHandle n;

  // get hardware port setting
  ros::NodeHandle("~").param("port", hardwarePortName, std::string("hardware_port_not_set"));

  // setup initial driver status
  status = DISABLED;
  armStatus = ARM_STATIC;
  hardwareStatus = HARDWARE_DISCONNECTED;

  // attempt to connect to hardware if the port was given
  if (hardwarePortName != "hardware_port_not_set")
  {
	if (false)//(connectHardware())
	  ROS_INFO("Successfully connected to hardware on port [%s]", hardwarePortName.c_str());
	else
	  ROS_INFO("Failed to connect to hardware using port [%s]", hardwarePortName.c_str());
  }

  // setup status publisher and timer
  statusPublisher = n.advertise<std_msgs::String>("status", 10);
  statusTicker = n.createTimer(ros::Duration(0.1), &FlatBotDriver::publishStatus, this);

  // setup joints of the arm
  joints.push_back(Joint("wrist_a_joint", 0, -90, 90));
  joints.push_back(Joint("elbow_joint", 0, -150, 150));
  joints.push_back(Joint("wrist_b_joint", 0, -90, 90));

  // setup status service server
  setStatusService = n.advertiseService("set_status", &FlatBotDriver::setStatus, this);

  // setup joint status publisher and create timer
  jointStatePublisher = n.advertise<sensor_msgs::JointState>("/joint_states", 50);
  jointTicker = n.createTimer(ros::Duration(0.02), &FlatBotDriver::publishJointState, this, false, true);

  // start the core robot control loop
  coreTimer = n.createTimer(ros::Duration(0.01), &FlatBotDriver::coreLoop, this);
}

void FlatBotDriver::publishStatus(const ros::TimerEvent&)
{
	std_msgs::String statusMsg;

	switch(status)
	{
	case DISABLED 		: statusMsg.data = "Disabled"; break;
	case LIVE_SIM 		: statusMsg.data = "Live Simulation"; break;
	case LIVE_HARDWARE 	: statusMsg.data = "Live Hardware"; break;
	}

	statusPublisher.publish(statusMsg);
}

bool FlatBotDriver::setStatus(flat_bot_msgs::SetStatus::Request  &req,
		              		  flat_bot_msgs::SetStatus::Response &res)
{
  if (req.newStatus == "disable")
  {
    status = DISABLED;
	res.result = "okay";
	return true;
  }
  if (req.newStatus == "enable hardware")
  {
	if (hardwareStatus == HARDWARE_CONNECTED)
	{
	  status = LIVE_HARDWARE;
	  res.result = "okay";
	}
   else
	{
	  res.result = "Cannot enable hardware mode, hardware not connected!";
	  ROS_WARN("FlatBot_Driver Warning: Attempt to enable hardware mode while hardware not connected.");
	}

	return true;
  }
  if (req.newStatus == "enable sim")
  {
	status = LIVE_SIM;
	res.result = "okay";
	return true;
  }

  res.result = "Error: unrecognised status request [" + req.newStatus + "]";
  ROS_WARN("FlatBot_Driver Warning: Request to setStatus service with unrecognised request [%s]", req.newStatus.c_str());
  return true;
}

void FlatBotDriver::publishJointState(const ros::TimerEvent& event)
{
  sensor_msgs::JointState jointStateMsg;

  jointStateMsg.header.stamp = event.current_real;

  //printf("joint state pub : time [%f]\n", event.current_real.toSec());

  for (int j=0; j<joints.size(); ++j)
  {
	jointStateMsg.name.push_back(joints[j].name);
	jointStateMsg.position.push_back(joints[j].pos);
	jointStateMsg.velocity.push_back(0.0);
  }

  jointStatePublisher.publish(jointStateMsg);
}

void FlatBotDriver::coreLoop(const ros::TimerEvent& event)
{
  //ros::Time curr = ros::Time::now();

	//ros::WallTime curr = ros::WallTime::now();

  //float secs = event.current_expected.toSec();

  //float angle = sin(secs/10.0);
  //joints[1].pos = angle;

  //printf("core_loop: time = [%f], angle = [%f]\n", secs, angle);

  if (status == LIVE_SIM)
  {
    // if there is a trajectory in progress then continue to simulate it
	if (armStatus == ARM_FOLLOWING_TRAJECTORY)
	{
	  // check that preempt has not been requested by the client
	  if (trajectoryActionServer.isPreemptRequested())
	  {
	    ROS_WARN("FlatBot_Driver Warning : Trajectory cancelled while in motion.");
	    trajectoryActionServer.setPreempted();
	    armStatus = ARM_STATIC;
	  }
	  else
	  {
 	    ros::Duration timeSinceStart = event.current_real - trajectoryStartTime;

        // check if the trajectory is finished and set last position if it is
 	    trajectory_msgs::JointTrajectoryPoint finalPoint;
 	    finalPoint = activeTrajectory.back();

	    if (timeSinceStart >= finalPoint.time_from_start)
	    {
		  for (int j=0; joints.size(); ++j)
		    joints[j].pos = finalPoint.positions[j];

		  // send success result to ExecuteTrajectory action server
		  moveit_msgs::ExecuteTrajectoryResult result;
		  result.error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
		  trajectoryActionServer.setSucceeded(result);

		  armStatus = ARM_STATIC;
	    }
	    else // if this is mid trajectory then interpolate joint positions for current point in time
	    {
		  trajectory_msgs::JointTrajectoryPoint startPoint, endPoint, interpolated;

		  for (int i=0; i<activeTrajectory.size()-1; ++i)
		  {
		    if (activeTrajectory[i  ].time_from_start >= timeSinceStart &&
			    activeTrajectory[i+1].time_from_start <  timeSinceStart)
		    {
		      startPoint = activeTrajectory[i];
		      endPoint = activeTrajectory[i+1];

		      float segmentDuration = (endPoint.time_from_start - startPoint.time_from_start).toSec();
		      float segmentOffset = (timeSinceStart - startPoint.time_from_start).toSec();
			  float weightStart = 1.0 - (segmentOffset / segmentDuration);

			  // interpolate adjacent points
			  for (int j=0; j<activeTrajectory[i].positions.size(); ++j)
			    interpolated.positions[j] = startPoint.positions[j] * weightStart + endPoint.positions[j] * (1-weightStart);
		    }
		  }

		  // set arm position
		  for (int j=0; j<joints.size(); ++j)
			joints[j].pos = interpolated.positions[j];

		  // publish movement status to Execute Trajectory action server
		  moveit_msgs::ExecuteTrajectoryFeedback trajectoryStatus;
		  trajectoryStatus.state = "Arm in motion";
		  trajectoryActionServer.publishFeedback(trajectoryStatus);
	    }
	  }
	}
  }

  else if (status == LIVE_HARDWARE)
  {

  }
}

void FlatBotDriver::trajectoryHandler(const moveit_msgs::ExecuteTrajectoryGoalConstPtr &goal)
{
  // check arm is capable of movement
  if (status != LIVE_SIM && status != LIVE_HARDWARE)
  {
	moveit_msgs::ExecuteTrajectoryResult result;

	result.error_code.val = moveit_msgs::MoveItErrorCodes::CONTROL_FAILED;
	ROS_WARN("FlatBot_Driver Warning : Trajectory sent while arm disabled.");
	trajectoryActionServer.setSucceeded(result);
	return;
  }

  // copy the robot trajectory to a property of the driver object and set the status to following trajectory
  activeTrajectory = goal->trajectory.joint_trajectory.points;
  trajectoryStartTime = ros::Time::now();
  armStatus = ARM_FOLLOWING_TRAJECTORY;
}

int main(int argc, char **argv)
{
	ROS_INFO("--[ FlatBot Driver node ]--");

	// setup ros for this node and get handle to ros system
	ros::init(argc, argv, "FlatBot_Driver");
	ros::start();

	FlatBotDriver driver;

    ROS_INFO("--[ FlatBot Driver node: startup complete ]--");

    ros::Rate loopRate(100);
    while(ros::ok())
    {
    	ros::spinOnce();
    	loopRate.sleep();

    	ros::Time c = ros::Time::now();
    	double secs = c.toSec();
    	float angle = sin(secs/2.0);
    	driver.joints[0].pos = angle;
    	driver.joints[1].pos = 0 - angle*2.0;
    	driver.joints[2].pos = angle;

    	//printf("time %f, time2 %f, angle %f\n", secs, c.toSec(), angle);
    }

	printf("--[ FlatBot Driver node: shutdown OK ]--\n");
	return 0;
}
