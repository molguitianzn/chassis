#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include "kinematics/kinematics.h"
#include <tf/transform_broadcaster.h>
#include <string.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include<sys/socket.h>
#include<linux/can.h>
#include<linux/can/raw.h>
#include <serial/serial.h>
#include "telecom/telecom_protocal.h"
#include <actionlib/client/simple_action_client.h>
#include "chassis_navigation/pathFollowingAction.h"

#define frameBufferSize 16
#define bufferLen 500
class chassis
{
public:
    chassis();
    ~chassis();

    // 接受到twist信息，转化为四个电机转速，然后发布
    void twistCb(const geometry_msgs::Twist::ConstPtr&);

    // 根据轮子计算里程计并发布
    void pubOdomtry();

    // 将计算出的四个轮子角速度发送
    void send2Motor(double*);

    // 获取四个轮子实际角速度
    int getFromMotor(double*);

    // 转化为轮速
    int rotateCode2double(u_int8_t* , double*);

    // can总线初始化
    void initCan();

    // 电机初始化
    void initMotor();

    // 电机关闭
    void stopMotor();

    // 判断电机是否故障
    bool isMotorOK();

    // 电机重新初始化
    void reInitMotor();

    // 发送一帧Can数据
    int sendCan(canid_t, u_int8_t*, u_int8_t);

    // uart 初始化
    void initUart();

    // uart接收
    void teleReceive();

    // 调用model2wheel和send2Motor()
    void send2base(double* model);

    // 里程计设置为指定数值
    // @param xyTheta: [0]: x(0); [1]: y(0); [2]: theta(from east to north in rad)
    void setOdom(double* xyTheta);

    // doneCB
    void pathFollowingDCB(const actionlib::SimpleClientGoalState& state, const chassis_navigation::pathFollowingResultConstPtr& result);

    // activeCB
    void pathFollowingACB();
    
    // feedbackCB
    void pathFollowingFCB(const chassis_navigation::pathFollowingFeedbackConstPtr& feedback);

protected:
    ros::NodeHandle nh;
    ros::Subscriber subTwist;
    ros::Publisher pubOdom;
    kinematics * ki;
    nav_msgs::Odometry odom;
    double current_time_s, last_time_s, dt;
    ros::Time current_time;
    double x, y, th, vx, vy, vth;
    geometry_msgs::TransformStamped transform;
    tf::TransformBroadcaster br;
    actionlib::SimpleActionClient<chassis_navigation::pathFollowingAction>* pathFollowingAction_;
    chassis_navigation::pathFollowingGoal goalPathFollowing;

    std::string canPort;
    int socketSend, nbytes;
    // int socketRec[4], nbytesR[4];
    struct can_frame frame;
    // struct can_frame frame_rec;
    std::string telePort;
    Telecom_Protocal teleProtocal;
    serial::Serial sp;
    unsigned char uartBuffer[bufferLen];
    double wheelLastTime[4];
};