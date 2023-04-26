#include "gnss_odom/gnss_odom.h"
#include <iostream>
#define R_earth 6371000.
#define pi 3.14159265
// #define deg2rad(x) x/180. * pi
#define deltaMetersE(lon, lon_ref, lat) (lon - lon_ref) / 360. * (R_earth * pi * 2.) * cos(deg2rad(lat))
#define deltaMetersN(lat, lat_ref) (lat - lat_ref) / 360. * (R_earth * pi * 2.)

double warp2mainRad(double rad)
{
    while(rad > pi) rad -= 2*pi;
    while(rad <= -1. * pi) rad += 2*pi;
    return rad;
}

double warp2main(double deg)
{
    while(deg > 180.) deg -= 360.;
    while(deg <= -180.) deg += 360.;
    return deg;
}
double deg2rad(double deg)
{
    deg = warp2main(deg);
    double ret = deg/180. * (pi);
}


GNSS_Odom::GNSS_Odom(): 
nh("~"), needInitialMap(false), needInitialOdom(true), canUpdate(false), debug_alone(false)
{
    nh.param<double>("gnss_rate", gnss_rate, 1.);
    nh.param<double>("v_min", v_min, 0.5);
    nh.param<bool>("debug_alone", debug_alone, false);
    nh.param<bool>("need_calibration", need_calibration, false);
    subGnrmc = nh.subscribe<gnss_driver::GNRMC>("/gnrmc", 10, &GNSS_Odom::gnssCb, this);
    subOdom = nh.subscribe<nav_msgs::Odometry>("/odom", 10, &GNSS_Odom::odomCb, this);
    pubGnssOdom = nh.advertise<nav_msgs::Odometry>("/gnssOdom", 10);
    askInitialPoseServer = nh.advertiseService("askInitialPose", &GNSS_Odom::askInitialPoseCb, this);

    if (debug_alone)
    {
        pubOdomPath = nh.advertise<nav_msgs::Path>("/OdomPath", 10);
        pubGnssPath = nh.advertise<nav_msgs::Path>("/GnssPath", 10);
        pubGnssOdomPath = nh.advertise<nav_msgs::Path>("/GnssOdomPath", 10);
    }

    F = Eigen::Matrix<double, 3, 3>::Identity();
    H = Eigen::Matrix<double, 3, 3>::Identity();
    M = Eigen::Matrix<double, 3, 3>::Identity();
    Q << 0.1, 0, 0, 0.1;
    R << 0.1, 0, 0, 0, 0.1, 0, 0, 0, 0.1;
    X << 0., 0., 0.;

    initKalman();
}

GNSS_Odom::~GNSS_Odom()
{
    
}

void GNSS_Odom::gnssCb(const gnss_driver::GNRMC::ConstPtr & pGnrmc)
{
    double deltaxy[2] = {0.};
    double yaw = 0;
    gnrmc[0] = *pGnrmc;
    nh.param<bool>("/need_init_map", needInitialMap, false);
    if (needInitialMap)
    {
        initialMap();
        gnrmc[1] = *pGnrmc;
        needInitialMap = false;
        nh.setParam("/need_init_map", needInitialMap);
    }
    observePose.position.x = deltaMetersE(gnrmc[0].longitude, MapOrigin[0], MapOrigin[1]);
    observePose.position.y = deltaMetersN(gnrmc[0].latitude, MapOrigin[1]);
    observePose.position.z = 0;
    deltaxy[0] = deltaMetersE(gnrmc[0].longitude, gnrmc[1].longitude, gnrmc[1].latitude);
    deltaxy[1] = deltaMetersN(gnrmc[0].latitude, gnrmc[1].latitude);
    gnrmc[1] = gnrmc[0];

    // if (sqrt(pow(deltaxy[0], 2) + pow(deltaxy[1], 2)) > 1. / gnss_rate * v_min) // may fail when gnss condition is poor
    if (odomOrigin.twist.twist.linear.x > 1. / gnss_rate * v_min)
    {
        // // method 1: get angle through differentiation of the positions
        // yaw = atan2(deltaxy[1], deltaxy[0]); // (-pi, pi]

        // method 2: get angle in gnrmc
        yaw = deg2rad(360. - 1. * gnrmc[0].yaw + 90.);

        observePose.orientation = tf::createQuaternionMsgFromYaw(yaw);
        measurement_update();
    }

    // for debug visualize
    if (debug_alone)
    {
        geometry_msgs::PoseStamped gnsspoint;
        gnsspoint.header.frame_id = "/map";
        gnsspoint.header.stamp = ros::Time::now();
        gnsspoint.pose = observePose;

        gnssPath.header.frame_id = "/map";
        gnssPath.header.stamp = ros::Time::now();
        gnssPath.poses.push_back(gnsspoint);
        pubGnssPath.publish(gnssPath);
    }
}

void GNSS_Odom::odomCb(const nav_msgs::Odometry::ConstPtr & pOdom)
{
    nh.param<bool>("/need_init_odom", needInitialOdom, false);
    if (needInitialOdom)
    {
        initialOdom(pOdom);
        needInitialOdom = false;
        nh.setParam("/need_init_odom", needInitialOdom);
        lastOdomts = pOdom->header.stamp;
    }

    if (need_calibration)
    {
        // for calibration use
        double coli = 0, coan = 0;
        nh.param<double>("coli", coli, 1.);
        nh.param<double>("coan", coan, 1.);
        odomCalibrate = *pOdom;
        odomCalibrate.twist.twist.linear.x *= coli;
        odomCalibrate.twist.twist.angular.z *= coan;
        tf::Quaternion quat;
        tf::quaternionMsgToTF(odomCalibrateLast.pose.pose.orientation, quat);
        double roll = 0, pitch = 0, yaw = 0;
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
        double deltaT = odomCalibrate.header.stamp.toSec() - odomCalibrateLast.header.stamp.toSec();
        odomCalibrate.pose.pose.position.x = odomCalibrateLast.pose.pose.position.x + deltaT * odomCalibrateLast.twist.twist.linear.x * cos(yaw);
        odomCalibrate.pose.pose.position.y = odomCalibrateLast.pose.pose.position.y + deltaT * odomCalibrateLast.twist.twist.linear.x * sin(yaw);
        yaw += deltaT * odomCalibrateLast.twist.twist.angular.z;
        yaw = warp2mainRad(yaw);
        odomCalibrate.pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
        odomCalibrateLast = odomCalibrate;
        odomOrigin = odomCalibrate;
    }
    else
    {
        odomOrigin = *pOdom;
    }

    predict();

    if (debug_alone)
    {
        // for debug visualize
        geometry_msgs::PoseStamped odompoint;
        odompoint.header.frame_id = "/odom";
        odompoint.header.stamp = ros::Time::now();
        odompoint.pose = odomOrigin.pose.pose;

        odomPath.header.frame_id = "/odom";
        odomPath.header.stamp = ros::Time::now();
        odomPath.poses.push_back(odompoint);
        pubOdomPath.publish(odomPath);
    }
}

void GNSS_Odom::initialMap()
{
    MapOrigin[0] = gnrmc[0].longitude;
    MapOrigin[1] = gnrmc[0].latitude;
}

void GNSS_Odom::initialOdom(const nav_msgs::Odometry::ConstPtr & pOdom)
{
    gnssOdom = *pOdom;

    if (need_calibration)
    {
        // for calibrition use
        double coli, coan;
        nh.param<double>("coli", coli, 1.);
        nh.param<double>("coan", coan, 1.);
        odomCalibrateLast = gnssOdom;
        odomCalibrateLast.twist.twist.linear.x *= coli;
        odomCalibrateLast.twist.twist.angular.z *= coan;
    }


    tf::Quaternion quat;
    tf::quaternionMsgToTF(pOdom->pose.pose.orientation, quat);
    double roll = 0, pitch = 0, yaw = 0;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    if (debug_alone)
    {
        nh.param<double>("initial_yaw", initial_yaw, 0);
        initial_yaw = deg2rad(initial_yaw);
        yaw += initial_yaw;
    }
    
    X << pOdom->pose.pose.position.x, pOdom->pose.pose.position.y, yaw;
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
    double nowTs;
    ros::Time nowT;
    if (!debug_alone)
    {
        nowT = odomOrigin.header.stamp;
    }
    else
    {
        nowT = ros::Time::now();
    }
    nowTs = nowT.toSec();
    double dT = nowTs - lastTs;
    L << dT * cos(X(2, 0)), 0, dT * sin(X(2, 0)), 0, 0, dT;
    U << odomOrigin.twist.twist.linear.x, odomOrigin.twist.twist.angular.z;
    X = Eigen::Matrix<double, 3, 3>::Identity() * X + L * U;
    X(2, 0) = warp2mainRad(X(2, 0));
    P = F * P * F.transpose() + L * Q * L.transpose();
    lastOdomts = nowT;
}

void GNSS_Odom::publishGnssOdom()
{
    current_time = ros::Time::now();
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(X(2, 0));
    transform.transform.rotation = odom_quat;
    transform.header.stamp = current_time;
    transform.header.frame_id = "gnssOdom";
    transform.child_frame_id = "base_footprint";
    transform.transform.translation.x = X(0, 0); transform.transform.translation.y = X(1, 0); transform.transform.translation.z = 0;
    br.sendTransform(transform);

    gnssOdom.child_frame_id = "base_footprint";
    gnssOdom.header.frame_id = "gnssOdom";
    gnssOdom.header.seq;
    gnssOdom.header.stamp = current_time;
    gnssOdom.pose.covariance;
    gnssOdom.pose.pose.orientation = odom_quat;
    gnssOdom.pose.pose.position.x = X(0, 0); gnssOdom.pose.pose.position.y = X(1, 0); gnssOdom.pose.pose.position.z = 0;
    gnssOdom.twist.covariance;
    gnssOdom.twist.twist.angular.x = 0; gnssOdom.twist.twist.angular.y = 0; gnssOdom.twist.twist.angular.z = odomOrigin.twist.twist.angular.z;
    gnssOdom.twist.twist.linear.x = odomOrigin.twist.twist.linear.x; gnssOdom.twist.twist.linear.y = 0; gnssOdom.twist.twist.linear.z = 0;
    pubGnssOdom.publish(gnssOdom);

    if (debug_alone)
    {
        // for debug visualize
        geometry_msgs::PoseStamped gnssodompoint;
        gnssodompoint.header.frame_id = "/map"; // /map->/odom应该重合
        gnssodompoint.header.stamp = ros::Time::now();
        gnssodompoint.pose = gnssOdom.pose.pose;

        gnssOdomPath.header.frame_id = "/map";
        gnssOdomPath.header.stamp = ros::Time::now();
        gnssOdomPath.poses.push_back(gnssodompoint);
        pubGnssOdomPath.publish(gnssOdomPath);
    }
}

bool GNSS_Odom::askInitialPoseCb(chassis_navigation::getInitialpos::Request& req, chassis_navigation::getInitialpos::Response& resp)
{
    
    resp.initialPose[0] = MapOrigin[0];
    resp.initialPose[1] = MapOrigin[1];
    return true;
}

