#include "path_follow_server/path_follow_server.h"

#define R_earth 6371000.
#define pi 3.14159265
// #define deg2rad(x) x/180. * pi
#define deltaMetersE(lon, lon_ref, lat) (lon - lon_ref) / 360. * (R_earth * pi * 2.) * cos(deg2rad(lat))
#define deltaMetersN(lat, lat_ref) (lat - lat_ref) / 360. * (R_earth * pi * 2.)

double deg2rad(double deg)
{
    while(deg >= 180.) deg -= 180.;
    while(deg < -180.) deg += 180.;
    double ret = deg/180. * (pi);
}

// bool NextGoal = true;
// void MBDoneCB(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result);
// void MBActiveCB();
// void MBFeedbackCB(const chassis_navigation::pathFollowingFeedbackConstPtr& feedback);

PathFollowServer::PathFollowServer():
nh("~"), nextGoal(true)
{
    askInitialPoseClient = nh.serviceClient<chassis_navigation::getInitialpos>("askInitialPose");
    ros::service::waitForService("askInitialPose");
    MBAClient_ = new actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>("/move_base", true); 
    if (!MBAClient_ -> waitForServer(ros::Duration(1)))
    {
        ROS_INFO("please launch move_base node!");
        return;
    }
    
    pathFollowingServer_ = new actionlib::SimpleActionServer<chassis_navigation::pathFollowingAction>(ros::NodeHandle(), "pathFollowing", boost::bind(&PathFollowServer::executePathFollowing, this, _1), false);
    pathFollowingServer_->start();
}

PathFollowServer::~PathFollowServer()
{
    if (pathFollowingServer_ != NULL)
        delete pathFollowingServer_;
}

void PathFollowServer::executePathFollowing(const chassis_navigation::pathFollowingGoalConstPtr& goal)
{
    ros::Rate r(5);
    chassis_navigation::pathFollowingFeedback feedback;
    bool flag;
    flag = askInitialPoseClient.call(askInitialPose);
    double x = 0, y = 0, yaw = 0, x_n = 0, y_n = 0;
    if (!flag)
    {
        pathFollowingServer_ -> setPreempted();
        return;
    }
    int lenth = goal -> longitudes.size();
    int i = 0;
    for(i = 0; i != lenth; i++)
    {
        x = deltaMetersE(goal->longitudes[i], askInitialPose.response.initialPose[0], askInitialPose.response.initialPose[1]);
        y = deltaMetersN(goal->latitudes[i], askInitialPose.response.initialPose[1]);
        MBGoal.target_pose.header.frame_id = "map";
        MBGoal.target_pose.header.stamp = ros::Time::now();
        MBGoal.target_pose.pose.position.x = x; MBGoal.target_pose.pose.position.y = y; MBGoal.target_pose.pose.position.z = 0;
        if (i = lenth - 1)
        {
            yaw = 0;
        }
        else
        {
            x_n = deltaMetersE(goal->longitudes[i+1], askInitialPose.response.initialPose[0], askInitialPose.response.initialPose[1]);
            y_n = deltaMetersN(goal->latitudes[i+1], askInitialPose.response.initialPose[1]);
            yaw = atan2(y_n - y, x_n - x);
        }
        MBGoal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
        nextGoal = false;
        

        MBAClient_ -> sendGoal(MBGoal, 
                            boost::bind(&PathFollowServer::MBDoneCB, this, _1, _2),
                            boost::bind(&PathFollowServer::MBActiveCB, this),
                            boost::bind(&PathFollowServer::MBFeedbackCB, this, _1));

        while(!nextGoal)
        {
            if (pathFollowingServer_ -> isPreemptRequested())
            {
                // canceled
                MBAClient_ -> cancelGoal();
                pathFollowingServer_ -> setPreempted();
                break;
            }
            r.sleep();
            ros::spinOnce();
        }
        
        feedback.progress_bar = i;
        pathFollowingServer_ -> publishFeedback(feedback);
    }
    pathFollowingServer_ -> setSucceeded();
}

// move_base doneCb
void PathFollowServer::MBDoneCB(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result)
{
    if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        nextGoal = true;
    }
}

// move_base activeCb
void PathFollowServer::MBActiveCB()
{

}

// move_base feedbackCb
void PathFollowServer::MBFeedbackCB(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback)
{
    feedback->base_position;
}