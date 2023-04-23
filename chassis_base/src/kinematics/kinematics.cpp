#include "kinematics/kinematics.h"

#define R 0.21 // 轮子半径，单位为m
#define D 0.81 // 左右两侧轮子间距(m)

kinematics::kinematics()
{
    wheel[0] = wheel[1] = wheel[2] = wheel[3] = 0;
    model[0] = model[1] = 0;
}

kinematics::~kinematics()
{

}

int kinematics::wheel2model(double * wheel)
{
    this->model[0] = (wheel[1]*R - wheel[0]*R) / 2.0; // 线速度 m/s
    this->model[1] = -1.0 * (wheel[0]*R + wheel[1]*R) / (D); // 角速度 rad/s
    // wheel[2] [3] 不考虑, 之后可以平均处理
    return 0;
}

int kinematics::model2wheel(double* model)
{
    this->wheel[0] = -1.0 * (2*model[0] + D*model[1]) / (2.0*R); // 右轮角速度，rad/s 
    this->wheel[1] = (2.0*model[0] - D*model[1]) / (2.0*R); // 左轮角速度, rad/s

    this->wheel[2] = this->wheel[1];
    this->wheel[3] = this->wheel[0];
    return 0;
}