#ifndef _GNSS_ODOM_H_
#define _GNSS_ODOM_H_
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include "gnss_driver/GNRMC.h"
#include "chassis_navigation/getInitialpos.h"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <nav_msgs/Path.h>
class GNSS_Odom
{
public:
    GNSS_Odom();
    ~GNSS_Odom();

    // 设置map系原点坐标
    void initialMap();

    // 设置里程计初始位姿
    void initialOdom(const nav_msgs::Odometry::ConstPtr & pOdom);

    // 接收到GNSS数据
    void gnssCb(const gnss_driver::GNRMC::ConstPtr & pGnrmc);

    // 接收到Odom数据
    void odomCb(const nav_msgs::Odometry::ConstPtr & pOdom);

    // gnss坐标转化到map坐标下
    void gnss2Map();

    // kalman初始化
    void initKalman();

    // kalman量测更新
    void measurement_update();

    // kalman预测
    void predict();

    // 发布gnss里程计融合结果 
    void publishGnssOdom();

    // 发布初始位置服务
    bool askInitialPoseCb(chassis_navigation::getInitialpos::Request& req, chassis_navigation::getInitialpos::Response& resp);

protected:
    gnss_driver::GNRMC gnrmc[2]; // 接收到的GNRMC [0]: current, [1] last
    nav_msgs::Odometry odomOrigin; // 原始里程计数据
    nav_msgs::Odometry gnssOdom; // kalman融合卫星和里程计
    ros::NodeHandle nh;
    ros::Subscriber subGnrmc;
    ros::Subscriber subOdom;
    ros::Publisher pubGnssOdom;
    ros::ServiceServer askInitialPoseServer;

// for debug use
    bool debug_alone; // for debug alone
    double initial_yaw; // for debug alone
    nav_msgs::Odometry odomCalibrate;
    nav_msgs::Odometry odomCalibrateLast;

    nav_msgs::Path odomPath;
    nav_msgs::Path gnssPath;
    nav_msgs::Path gnssOdomPath;
    ros::Publisher pubOdomPath;
    ros::Publisher pubGnssPath;
    ros::Publisher pubGnssOdomPath;

// if need calibration
    bool need_calibration;

protected:
    double MapOrigin[2]; // 原点坐标[E, N]
    geometry_msgs::TransformStamped transMap2base;
    geometry_msgs::TransformStamped transform; // map->odom
    tf::TransformBroadcaster br;
    bool needInitialMap; 
    bool needInitialOdom;
    double gnss_rate; // 卫星频率，用于轨迹差分求方向
    double v_min; // 当车速大于这个时，用前后两个采样点的差分求解方向
    geometry_msgs::Pose observePose;
    bool canUpdate; // 能够测量出车速航向，能够进行量测更新

protected:
    Eigen::Matrix<double, 3, 1> X; // 
    Eigen::Matrix<double, 3, 1> Y; // 
    Eigen::Matrix<double, 2, 1> U; // 
    Eigen::Matrix<double, 3, 3> F; // const
    Eigen::Matrix<double, 3, 2> L; // 
    Eigen::Matrix<double, 3, 3> H; // const
    Eigen::Matrix<double, 3, 3> M; // const
    Eigen::Matrix<double, 3, 3> P; // 
    Eigen::Matrix<double, 3, 3> K; // 
    Eigen::Matrix<double, 2, 2> Q; // const
    Eigen::Matrix<double, 3, 3> R; // const

    ros::Time lastOdomts; // 上一帧里程计的时间戳
    ros::Time current_time; // 当前时刻

};
#endif