#include <ros/ros.h>
#include <std_msgs/String.h>
#include <irobot_create_2_1/SensorPacket.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <sstream>

bool isBumpedRight = false; 
bool isBumpedLeft = false; 

void chatterCallback(const irobot_create_2_1::SensorPacket msg)
{		
	isBumpedRight = msg.bumpRight; 
	isBumpedLeft = msg.bumpLeft; 

	ROS_INFO("Right: %d\tLeft: %d", isBumpedRight, isBumpedLeft);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "controller");

	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("sensorPacket", 1000, chatterCallback);	
	ros::Publisher chatter_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 20);
	

	ros::Rate loop_rate(10);

	while (ros::ok())
	{
		geometry_msgs::Twist msg;

		if (isBumpedRight == false &&
			isBumpedLeft == false)		
		{
			msg.linear.x = 0.5;
		}		
		else if (isBumpedRight)
		{
			msg.angular.z = 45;  
		}
		else if (isBumpedLeft)
		{
			msg.angular.z = -45;
		}


		// print info on screen 
		ROS_INFO("Linear X: %i", msg.linear.x);
		
		// publish message
		chatter_pub.publish(msg);

		ros::spinOnce();
		loop_rate.sleep();
	}

	ros::spin();

	return 0;
}
