#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"

void scanValues(const sensor_msgs::LaserScan laser)
{
    ROS_INFO("size[%d]: ", laser.ranges.size());

    for (unsigned int i=0; i<laser.ranges.size();i++)
    {
        ROS_INFO("intens[%f]: ", laser.ranges[i]);

    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "LaserController");
    ros::NodeHandle n;
    ros::Subscriber hokuyoSubscriber = n.subscribe("/scan", 1, scanValues);
    ros::spin();
    return 0;
}
