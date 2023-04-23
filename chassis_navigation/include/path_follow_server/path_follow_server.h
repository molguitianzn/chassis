#ifndef _GNSS_ODOM_H_
#define _GNSS_ODOM_H_
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <actionlib/server/simple_action_server.h>
#include "chassis_navigation/pathFollowingAction.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <chassis_navigation/getInitialpos.h>

class PathFollowServer
{
public:
    PathFollowServer();
    ~PathFollowServer();

    // 路径跟踪服务
    void executePathFollowing(const chassis_navigation::pathFollowingGoalConstPtr& goal);

    // move_base doneCb
    void MBDoneCB(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result);

    // move_base activeCb
    void MBActiveCB();

    // move_base feedbackCb
    void MBFeedbackCB(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback);
    
protected:
    ros::NodeHandle nh;
    move_base_msgs::MoveBaseGoal MBGoal;
    ros::ServiceClient askInitialPoseClient;
    chassis_navigation::getInitialpos askInitialPose;
    actionlib::SimpleActionServer<chassis_navigation::pathFollowingAction>* pathFollowingServer_;
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>* MBAClient_;
    bool nextGoal;
};


#endif