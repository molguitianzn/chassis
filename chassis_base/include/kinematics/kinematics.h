
class kinematics
{
public:
    kinematics();
    ~kinematics();

    // 左右轮子转速转换为车辆运动线速度和角速度
    // wheel: double wheel[2] = {左侧轮角速度，右侧轮子角速度}，前进为正，倒退为负,单位rad/s
    // model: double model[2] = {车辆线速度，旋转角速度}：线速度前进为正，倒退为负,单位m/s；角速度逆时针为正数, 顺时针为负，单位rad/s
    int wheel2model(double * wheel);

    // 车辆线速度角速度转换为轮子转速
    // wheel: double wheel[2] = {左侧轮角速度，右侧轮子角速度}，前进为正，倒退为负,单位rad/s
    // model: double model[2] = {车辆线速度，旋转角速度}：线速度前进为正，倒退为负,单位m/s；角速度逆时针为正数, 顺时针为负，单位rad/s
    int model2wheel(double* model);
    
    double wheel[4]; // wheel[0]左侧前轮转速; 1右侧前轮转速; 2右侧后轮转速; 3左侧后轮转速 (rad/s, 轮子正面看逆时针为正)
    double model[2]; // 0: 线速度 (m/s); 1: 角速度 (rad/s)
};