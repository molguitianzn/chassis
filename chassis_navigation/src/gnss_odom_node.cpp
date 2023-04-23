#include <ros/ros.h>
#include "gnss_odom/gnss_odom.h"

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "gnss_odom_node");
    GNSS_Odom gnss_odom;
    int freq = 100;
    ros::Rate rate(freq);
    ROS_INFO("\033[1;32m---->\033[0m gnss_odom node started.");
    while(ros::ok())
    {
        ros::spinOnce();
        gnss_odom.publishGnssOdom();
        rate.sleep();
    }
    ros::spin();
    return 0;
}