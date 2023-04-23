#include <ros/ros.h>
#include "gnss_driver/gnss_driver.h"
int main(int argc, char* argv[])
{
    ros::init(argc, argv, "gnss_driver_node");
    ROS_INFO("\033[1;32m---->\033[0m gnss driver node started.");
    GNSS_Driver gnss_Driver;
    int freq = 10;
    ros::Rate rate(freq);
    while(ros::ok())
    {
        ros::spinOnce();
        gnss_Driver.gnssReceive();
        rate.sleep();
    }
    ros::spin();
    return 0;
}