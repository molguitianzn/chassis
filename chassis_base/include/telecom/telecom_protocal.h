#ifndef _TELECOM_PROTOCAL_
#define _TELECOM_PROTOCAL_
#include <ros/ros.h>
#include <vector>
class Telecom_Protocal
{
public:
    Telecom_Protocal();
    ~Telecom_Protocal();
    void parse_buffer(uint8_t* buf, int);
    bool CRC_cal(uint8_t*, int);
protected:
    // @param buf: 数据堆栈区
    // @param i: 当前读取位置
    // @param byte_num: 需要读取位数
    // @param Dest: 目标对象的在内存中的首地址
    void readBytes(uint8_t* buf, int* i, int byte_num, void* Dest);
public:
    bool autoOrManual; // false: manual, true: auto
    char functionCode; // 1: 手动, 2自动, 3电机重启
    char autoFunctionCode;
    double model[2]; // vx in manual mode; wz in manual mode
    double initAngle; // 自动模式初始航向角，东偏北多少rad
    std::vector<double> longitudes; // positive: E, nagative: W
    std::vector<double> latitudes; // positive:N, nagative: S
};

#endif
