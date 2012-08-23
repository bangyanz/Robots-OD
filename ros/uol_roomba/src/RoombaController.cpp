#include <ros/ros.h>
#include <std_msgs/String.h>
#include <irobot_create_2_1/SensorPacket.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <sstream>

boolean isBumpedLeft;
boolean isBumpedRight;

void chatterCallback(const irobot_create_2_1::SensorPacket msg)
{		
	ROS_INFO("Right: %d\tLeft: %d", msg.bumpRight, msg.bumpLeft);
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

		msg.linear.x = 1.0;
		ROS_INFO("Linear X: %i", msg.linear.x);

		chatter_pub.publish(msg);

		ros::spinOnce();
		loop_rate.sleep();
	}

	ros::spin();

	return 0;
}
