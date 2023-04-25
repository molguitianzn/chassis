#ifndef _GNSS_DRIVER_H_
#define _GNSS_DRIVER_H_
#include <ros/ros.h>
#include <serial/serial.h>
#include "gnss_driver/GNGGA.h"
#include "gnss_driver/GNRMC.h"
class GNSS_Driver
{
public:
    GNSS_Driver();
    ~GNSS_Driver();

    // uart 初始化
    void initUart();

    // uart接收gnss
    void gnssReceive();

    // 解析GNSS
    int parser(char *data, bool * isRMC);

protected:
    gnss_driver::GNGGA gngga;
    gnss_driver::GNRMC gnrmc;
    ros::NodeHandle nh;
    ros::Publisher pubGngga;
    ros::Publisher pubGnrmc;
    ros::Time current_time;
    std::string telePort;
    serial::Serial sp;
    unsigned char uartBuffer[4096];
};

#endif