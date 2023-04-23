#include <ros/ros.h>
#include "path_follow_server/path_follow_server.h"

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "chassis_navigation_node");
    PathFollowServer pathFollowServer;
    ROS_INFO("\033[1;32m---->\033[0m chassis navigation node started!.");
    ros::spin();
    return 0;
}