#include <ros/ros.h>
#include "chassis/chassis.h"

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "kinematics_node");
    
    chassis chassisBase;
    // subscribe: geometry_msgs::Twist cmd_vel, send instructions via can bus
    // publish: geometry_msgs::Odom odom with the data get from can bus
    int freq = 100;
    ros::Rate rate(freq);
    ROS_INFO("\033[1;32m---->\033[0m chassis base node started.");
    while(ros::ok())
    {
        ros::spinOnce();
        chassisBase.pubOdomtry();
        chassisBase.teleReceive();
        rate.sleep();
    }
    ros::spin();
    return 0;
}
