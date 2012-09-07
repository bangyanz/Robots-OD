#include <ros/ros.h>
#include <std_msgs/String.h>
#include <irobot_create_2_1/SensorPacket.h>

#include <geometry_msgs/Twist.h>
#include <iostream>
#include <sstream>

#include "sensor_msgs/LaserScan.h"


irobot_create_2_1::SensorPacket SensorPacketmsg;
geometry_msgs::Twist Twistmsg;

// used for checking sensor states
bool isCharging = true;
bool isBumpLeft = false;
bool isBumpRight = false;

bool isObjectDetected = false;


// Different states 
enum States
{
	Charging, 
	NotCharging, 
	Moving, 
	BumpReaction
};

 
enum Behaviours
{
	// Charging Behaviours
	Sleep,

	// Not Charging Behaviours
	Awake,
	
	// Moving Behaviours
	Forward, 
	Backward, 
	ArcRight, 
	ArcLeft, 
	SpinLeft, 
	SpinRight,

	// Bump reaction
	ObstacleDetected
};

States robotState = Charging;
Behaviours robotBehaviour = Sleep; 

// Method to be run when robotState = Charging
void charging(Behaviours _Behaviour)
{
	// check that the right state has been called
	if (robotState != Charging)
		return; 	// If not exit the function
	

	// Check Behaviours
	switch (_Behaviour)
	{
		case Sleep:
			// Check to see if we are still charging
			if (!isCharging)
			{
				// now we can change states
				robotState = NotCharging; 
				robotBehaviour = Awake; 
				ROS_INFO("Status = Not Charging"); 
			}
			break;	
	}
}

void notCharging(Behaviours _Behaviour)
{
	// check that the right state has been called
	if (robotState != NotCharging)
		return; 

	// Check behaviours
	switch(_Behaviour)
	{
		
		case Awake:
			// set to moving state
			robotState = Moving; 
			robotBehaviour = Forward; 
			break; 
			
	}
}

void moving(Behaviours _Behaviour)
{
	// check that the right state has been called
		if (robotState != Moving)
			return;

		// Check behaviours
		switch(_Behaviour)
		{
		
			case Forward:
				if((isBumpLeft || isBumpRight) || isObjectDetected)
				{
					robotState = BumpReaction; 
					robotBehaviour = ObstacleDetected; 
					break;
				}
				Twistmsg.linear.x = 10;
				Twistmsg.angular.z = 0; 
				
				break; 			
		}
}


void bumpReaction(Behaviours _Behaviour)
{
	// check that the right state has been called
		if (robotState != BumpReaction)
			return;

	switch(_Behaviour)
	{
		case ObstacleDetected:
			Twistmsg.linear.x = 0; 
			if (isBumpLeft)
				Twistmsg.angular.z = -45; 
			else if (isBumpRight)
				Twistmsg.angular.z = 45;
			else if (isObjectDetected)
			{
				Twistmsg.angular.z = 20;
			}

			
			robotState = Moving; 
			robotBehaviour = Forward; 
			break; 
	}
}


// Update sensor information
void chatterCallback(const irobot_create_2_1::SensorPacket _SensorPacketmsg)
{	
	// update sensor information
	SensorPacketmsg = _SensorPacketmsg;

	// update variables
	isCharging = SensorPacketmsg.internalCharger;
	isBumpLeft = SensorPacketmsg.bumpLeft;
	isBumpRight = SensorPacketmsg.bumpRight;
}



void scanValues(const sensor_msgs::LaserScan laser)
{
    //ROS_INFO("size[%d]: ", laser.ranges.size());

    for (unsigned int i=0; i<laser.ranges.size();i++)
    {
	if (laser.ranges[i] < 0.35 && laser.ranges[i] > 0.1)
	{
		isObjectDetected = true;
        	ROS_INFO("intens[%f] [%i]: ", laser.ranges[i], i);
		return;
	}
	else
	{
		isObjectDetected = false;
	}
    }
}




int main(int argc, char **argv)
{

	
	// Set State to Running by default
	robotState = Charging;

	// Initialize - give the talker a name 
	ros::init(argc, argv, "controller");
	
	// Create node to handle messages
	ros::NodeHandle n;
	
	// Create subscriber to listen to messages on "sensorPacket" channel
	// 1000 messages to store before deleting older messages
	// When a message is received invoke function 'chatterCallback' 
	ros::Subscriber sub = n.subscribe("sensorPacket", 1000, chatterCallback);
	
	// Create subscriber to listen to messages on "/scan" topic
	ros::Subscriber hokuyoSubscriber = n.subscribe("/scan", 100, scanValues);	

	// Create Publisher to send messages on "cmd_vel" channel
	// Set buffer size to 20
	ros::Publisher pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 20);
	
	// Loop 10 times a second
	ros::Rate loop_rate(10);

	while (ros::ok())
	{
		

		// stop if charged
		ROS_INFO("Status = %i", robotState); 	
		if (isCharging)
		{
			// now we can change states
			robotState = Charging; 
			robotBehaviour = Sleep; 
			Twistmsg.linear.x = 0; 
			Twistmsg.angular.z = 0; 
		}
		

		switch (robotState)
		{
			case Charging:
				charging(robotBehaviour);
				break;
			case NotCharging:
				notCharging(robotBehaviour);
				break;
			case Moving:
				moving(robotBehaviour);
				break;
			case BumpReaction:
				bumpReaction(robotBehaviour); 
				break;
			
		}



		// publish message		
		pub.publish(Twistmsg);
		
		// Send/Look for messages 
		loop_rate.sleep();

		ros::spinOnce();
	}
	
	ros::spin();

	return 0;
}
