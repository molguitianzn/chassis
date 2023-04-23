#include "gnss_odom/gnss_odom.h"

#define R_earth 6371000.
#define pi 3.14159265
#define deg2rad(x) x/180. * pi
#define deltaMetersE(lon, lon_ref, lat) (lon - lon_ref) / 360. * (R_earth * pi * 2.) * cos(deg2rad(lat))
#define deltaMetersN(lat, lat_ref) (lat - lat_ref) / 360. * (R_earth * pi * 2.)
GNSS_Odom::GNSS_Odom(): 
nh("~"), needInitialMap(true), needInitialOdom(true), canUpdate(false)
{
    nh.param<double>("gnss_rate", gnss_rate, 1.);
    nh.param<double>("v_min", v_min, 0.5);
    subGnrmc = nh.subscribe<gnss_driver::GNRMC>("/gnrmc", 10, &GNSS_Odom::gnssCb, this);
    subOdom = nh.subscribe<nav_msgs::Odometry>("/odom", 10, &GNSS_Odom::odomCb, this);
    pubGnssOdom = nh.advertise<nav_msgs::Odometry>("/gnssOdom", 10);
    askInitialPoseServer = nh.advertiseService("askInitialPose", &GNSS_Odom::askInitialPoseCb, this);

    F = Eigen::Matrix<double, 3, 3>::Identity();
    H = Eigen::Matrix<double, 3, 3>::Identity();
    M = Eigen::Matrix<double, 3, 3>::Identity();
    Q << 0.1, 0, 0, 0.1;
    R << 0.1, 0, 0, 0, 0.1, 0, 0, 0, 0.1;

    initKalman();
    
    lastOdomts = ros::Time::now();

    // pathFollowingServer_ = new actionlib::SimpleActionServer<chassis_navigation::pathFollowingAction>(ros::NodeHandle(), "pathFollowing", boost::bind(&GNSS_Odom::executePathFollowing, this, _1), false);
    // pathFollowingServer_->start();
}

GNSS_Odom::~GNSS_Odom()
{
    // if (pathFollowingServer_ != NULL)
    //     delete pathFollowingServer_;
}

void GNSS_Odom::gnssCb(const gnss_driver::GNRMC::ConstPtr & pGnrmc)
{
    double deltaxy[2] = {0.};
    double yaw = 0;
    gnrmc[0] = *pGnrmc;
    if (needInitialMap)
    {
        initialMap();
        gnrmc[1] = *pGnrmc;
        needInitialMap = false;
    }
    observePose.position.x = deltaMetersE(gnrmc[0].longitude, MapOrigin[0], MapOrigin[1]);
    observePose.position.y = deltaMetersN(gnrmc[0].latitude, MapOrigin[1]);
    observePose.position.z = 0;
    deltaxy[0] = deltaMetersE(gnrmc[0].longitude, gnrmc[1].longitude, gnrmc[1].latitude);
    deltaxy[1] = deltaMetersN(gnrmc[0].latitude, gnrmc[1].longitude);
    gnrmc[1] = gnrmc[0];

    if (sqrt(pow(deltaxy[0], 2) + pow(deltaxy[1], 2)) > 1. / gnss_rate * v_min)
    {
        yaw = atan2(deltaxy[1], deltaxy[0]);
        observePose.orientation = tf::createQuaternionMsgFromYaw(yaw);
        measurement_update();
    }
}

void GNSS_Odom::odomCb(const nav_msgs::Odometry::ConstPtr & pOdom)
{
    odomOrigin = *pOdom;
    if (needInitialOdom)
    {
        initialOdom();
        needInitialOdom = false;
    }
    predict();
}

void GNSS_Odom::initialMap()
{
    MapOrigin[0] = gnrmc[0].longitude;
    MapOrigin[1] = gnrmc[0].latitude;
}

void GNSS_Odom::initialOdom()
{
    gnssOdom = odomOrigin;
    tf::Quaternion quat;
    tf::quaternionMsgToTF(odomOrigin.pose.pose.orientation, quat);
    double roll = 0, pitch = 0, yaw = 0;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    X << odomOrigin.pose.pose.position.x, odomOrigin.pose.pose.position.y, yaw;
    initKalman();
}

void GNSS_Odom::gnss2Map()
{
    geometry_msgs::Quaternion map_quat = tf::createQuaternionMsgFromYaw(gnrmc[0].yaw);
    transMap2base.transform.rotation = map_quat;
    transMap2base.header.stamp = ros::Time::now();
    transMap2base.header.frame_id = "map";
    transMap2base.child_frame_id = "base_footprint";
    transMap2base.transform.translation.x = gnrmc[0].longitude - MapOrigin[0];
}

void GNSS_Odom::initKalman()
{
    P = Eigen::Matrix<double, 3, 3>::Identity();
}

void GNSS_Odom::measurement_update()
{
    Eigen::Matrix<double, 3, 1> y_check;
    Eigen::Matrix<double, 3, 1> y_hat;
    tf::Quaternion quat;
    tf::quaternionMsgToTF(observePose.orientation, quat);
    double roll = 0, pitch = 0, yaw = 0;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    y_hat << observePose.position.x, observePose.position.y, yaw;
    K = P * H.transpose() * (H * P * H.transpose() + M * R * M.transpose()).inverse();
    y_check = H * X;
    X = X + K * (y_hat - y_check);
    P = (Eigen::Matrix<double, 3, 3>::Identity() - K * H) * P;
}

void GNSS_Odom::predict()
{
    double lastTs = lastOdomts.toSec();
    double nowTs = odomOrigin.header.stamp.toSec();
    double dT = nowTs - lastTs;
    L << dT * cos(X(3, 1)), 0, dT * sin(X(3, 1)), 0, 0, dT;
    U << odomOrigin.twist.twist.linear.x, odomOrigin.twist.twist.angular.z;
    X = Eigen::Matrix<double, 3, 3>::Identity() * X + L * U;
    P = F * P * F.transpose() + L * Q * L.transpose();
    lastOdomts = odomOrigin.header.stamp;
}

void GNSS_Odom::publishGnssOdom()
{
    current_time = ros::Time::now();
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(X(3, 1));
    transform.transform.rotation = odom_quat;
    transform.header.stamp = current_time;
    transform.header.frame_id = "gnssOdom";
    transform.child_frame_id = "base_footprint";
    transform.transform.translation.x = X(1, 1); transform.transform.translation.y = X(2, 1); transform.transform.translation.z = 0;
    br.sendTransform(transform);

    gnssOdom.child_frame_id = "base_footprint";
    gnssOdom.header.frame_id = "gnssOdom";
    gnssOdom.header.seq;
    gnssOdom.header.stamp = current_time;
    gnssOdom.pose.covariance;
    gnssOdom.pose.pose.orientation = odom_quat;
    gnssOdom.pose.pose.position.x = X(1, 1); gnssOdom.pose.pose.position.y = X(2, 1); gnssOdom.pose.pose.position.z = 0;
    gnssOdom.twist.covariance;
    gnssOdom.twist.twist.angular.x = 0; gnssOdom.twist.twist.angular.y = 0; gnssOdom.twist.twist.angular.z = odomOrigin.twist.twist.angular.z;
    gnssOdom.twist.twist.linear.x = odomOrigin.twist.twist.linear.x; gnssOdom.twist.twist.linear.y = 0; gnssOdom.twist.twist.linear.z = 0;
    pubGnssOdom.publish(gnssOdom);
}

bool GNSS_Odom::askInitialPoseCb(chassis_navigation::getInitialpos::Request& req, chassis_navigation::getInitialpos::Response& resp)
{
    
    resp.initialPose[0] = MapOrigin[0];
    resp.initialPose[1] = MapOrigin[1];
    return true;
}

// void GNSS_Odom::executePathFollowing(const chassis_navigation::pathFollowingGoalConstPtr& goal)
// {
//     ros::Rate r(1);
//     chassis_navigation::pathFollowingFeedback feedback;
//     int lenth = goal -> longitudes.size();
//     int i = 0;
//     for(i = 0; i != lenth; i++)
//     {
//         if (pathFollowingServer_ -> isPreemptRequested())
//         {
//             // canceled
//             pathFollowingServer_ -> setPreempted();
//             break;
//         }
//         feedback.progress_bar = 1;
//         pathFollowingServer_ -> publishFeedback(feedback);
//         r.sleep();
//     }
//     pathFollowingServer_ -> setSucceeded();
// }


