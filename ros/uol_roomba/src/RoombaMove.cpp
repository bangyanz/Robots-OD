#include <ros/ros.h>
#include <std_msgs/String.h>
#include <irobot_create_2_1/SensorPacket.h>
#include <geometry_msgs/Twist.h>
#include <iostream>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "cmd_vel");

	ros::NodeHandle n;

	ros::Publisher chatter_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 20);	
	ros::Rate loop_rate(10);

	int count = 0;
	while (ros::ok())
	{
	    	geometry_msgs::Twist msg;

		//msg.linear.x = 1.0;

		msg.angular.x = 0.0;
		msg.angular.y = 0.0;
		msg.angular.z = 0.0;
	

	    	ROS_INFO("Sending XY to /cmd_vel");

	    	chatter_pub.publish(msg);

	    	ros::spinOnce();

	    	loop_rate.sleep();
		count++;
	  }

	return 0;
}
