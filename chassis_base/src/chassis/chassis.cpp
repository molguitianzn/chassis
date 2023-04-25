#include "chassis/chassis.h"


#define pi 3.14159265
#define RR 10.0    // reduction ratio, motor / wheel

chassis::chassis(): nh("~")
{
    nh.param<std::string>("telecom_port", telePort, "/dev/ttyS0");
    nh.param<std::string>("can_port", canPort, "can0");
    nh.param<bool>("debug_alone", debug_alone, "false");
    // nh.param<bool>("autoOrManual", teleProtocal.autoOrManual, "true");
    

    ki = (kinematics*)malloc(sizeof(kinematics));
    subTwist = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 10, &chassis::twistCb, this);
    pubOdom = nh.advertise<nav_msgs::Odometry>("/odom", 10);
    current_time = ros::Time::now();
    last_time_s = current_time.toSec();
    x = y = th = vx = vy = vth = 0;
    wheelLastTime[0] = 0; wheelLastTime[1] = 0;
    wheelLastTime[2] = 0; wheelLastTime[3] = 0;
    initUart();
    initCan();
    initMotor();

    if (debug_alone)
    {
        return;
    }

    pathFollowingAction_ = new actionlib::SimpleActionClient<chassis_navigation::pathFollowingAction>("pathFollowing", true);
    ROS_INFO("waiting for path following server to start");
    pathFollowingAction_ -> waitForServer();
    ROS_INFO("following server started!");
}

chassis::~chassis()
{
    stopMotor();
    close(socketSend);
    sp.close();
    // for (int i = 0; i != 4; i++)
    // {
    //     close(socketRec[i]);
    // }
    free(ki);
}

void chassis::initCan()
{  
    struct sockaddr_can addr;    
    struct ifreq ifr;       
    // struct can_filter rfilter[1];

    socketSend = socket(PF_CAN, SOCK_RAW, CAN_RAW);//创建套接字    
    strcpy(ifr.ifr_name, this->canPort.c_str());    
    ioctl(socketSend, SIOCGIFINDEX, &ifr);//指定can0设备    
    addr.can_family = AF_CAN;    
    addr.can_ifindex = ifr.ifr_ifindex;    
    bind(socketSend,(struct sockaddr*)&addr, sizeof(addr));//将套接字与can0绑定   
    setsockopt(socketSend, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0); 
}

void chassis::initUart()
{
    try
    {
        sp.setPort(telePort.c_str());
        sp.setBaudrate(115200);
        sp.setBytesize(serial::bytesize_t::eightbits);
        sp.setStopbits(serial::stopbits_t::stopbits_one);
        sp.setFlowcontrol(serial::flowcontrol_t::flowcontrol_none);

        serial::Timeout time_out = serial::Timeout::simpleTimeout(1000);

        sp.setTimeout(time_out);
        sp.open();
    }
    catch(serial::IOException & e)
    {
        ROS_ERROR_STREAM("Unable to open telecom port!");
    }
    if (!sp.isOpen())
    {
        ROS_ERROR_STREAM("Unable to open telecom port!");
    }
}

int chassis::sendCan(canid_t canid, u_int8_t* data, u_int8_t len)
{
    memset((void*)&frame, 0x00, sizeof(frame));
    frame.can_id = canid;
    frame.can_dlc = len;
    for (int i = 0; i < len; i++)
    {
        frame.data[i] = data[i];
    }
    nbytes = write(socketSend, &frame, sizeof(frame));
    usleep(1000); // delay 1ms
    if (nbytes != sizeof(frame))
    {
        return 0; // error
    }
    return 1; // OK
}

void chassis::initMotor()
{
    uint8_t data[8] = {0};
    
    data[0] = 0x01;
    sendCan(0, data, 2);
    memset(data, 0x00, 8);

    for (int i = 0; i != 4; i++)
    {
        // 将电机配置为速度模式
        data[0] = 0x2f; data[1] = data[2] = 0x60; data[4] = 0x03;
        sendCan(0x601+i, data, 8);
        memset(data, 0x00, 8);

        // 使能电机驱动
        data[0] = 0x2b; data[1] = 0x40; data[2] = 0x60; data[4] = 0x0f;
        sendCan(0x601+i, data, 8);
        memset(data, 0x00, 8);

        // 速度设置为0 
        data[0] = 0x23; data[1] = 0xff; data[2] = 0x60;
        sendCan(0x601+i, data, 8);
        memset(data, 0x00, 8);
    }
}

void chassis::stopMotor()
{
    uint8_t data[8] = {0};

    for (int i = 0; i != 4; i++)
    {
        // 速度设置为0 
        data[0] = 0x23; data[1] = 0xff; data[2] = 0x60;
        sendCan(0x601+i, data, 8);
        memset(data, 0x00, 8);
    }

    // 解决报错
    data[0] = 0x81; data[1] = 0;
    sendCan(0, data, 2);
    memset(data, 0, 8);
    
    for (int i = 0; i != 4; i++)
    {
        // 断开使能
        data[0] = 0x2b; data[1] = 0x40; data[2] = 0x60;
        sendCan(0x601+i, data, 8);
        memset(data, 0x00, 8);
    }

}

bool chassis::isMotorOK()
{
    bool flag = true;
    int ss, nbytes, nbytes2, sr[4];    
    struct sockaddr_can addr;    
    struct ifreq ifr;    
    struct can_frame frameQ, frame2;    
    struct can_filter rfilter2[1];

    ss = socket(PF_CAN, SOCK_RAW, CAN_RAW);//创建套接字    
    strcpy(ifr.ifr_name, this->canPort.c_str());    
    ioctl(ss, SIOCGIFINDEX, &ifr);//指定can0设备    
    addr.can_family = AF_CAN;    
    addr.can_ifindex = ifr.ifr_ifindex;    
    bind(ss,(struct sockaddr*)&addr, sizeof(addr));//将套接字与can0绑定   
    setsockopt(ss, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0); 

    for (int i = 0; i != 4; i++)
    {
        sr[i] = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        ioctl(sr[i], SIOCGIFINDEX, &ifr);
        bind(sr[i], (struct sockaddr*)&addr, sizeof(addr));
        rfilter2[0].can_id = 0x581 + i;
        rfilter2[0].can_mask = CAN_SFF_MASK;
        setsockopt(sr[i], SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter2, sizeof(rfilter2));
    }
    
    for (int i = 0; i != 4; i++)
    {
        memset((void*)&frameQ.data, 0x00, sizeof(frameQ.data));
        frameQ.can_id = 0x601 + i;
        frameQ.can_dlc = 8;
        frameQ.data[0] = 0x40; frameQ.data[1] = 0x83; frameQ.data[2] = 0x21;
        memset((void*)&frame2, 0, sizeof(frame2));
        nbytes = write(ss, &frameQ, sizeof(frameQ));
        if(nbytes != sizeof(frameQ))        
        {            
            printf("Send Error frame[0]\n!");            
            break;//发送错误，退出        
        }    
        usleep(1000);
        nbytes2 = read(sr[i], &frame2, sizeof(frame2));
        if(frame2.can_dlc == 8 && frame2.can_id == (0x581+i))
        {            
            if (
                frame2.data[0] != 0x43 ||
                frame2.data[1] != 0x83 ||
                frame2.data[2] != 0x21 ||
                frame2.data[3] != 0 ||
                frame2.data[4] != 0 ||
                frame2.data[5] != 0 ||
                frame2.data[6] != 0 ||
                frame2.data[7] != 0
            )
            {
                flag = false;
            }
        } 
    }

    // for(int i = 0; i != 4; i++)
    // {
    //     printf("wheel[%d]: %lf  ", i, wheel[i]);
    // }
    // printf("\r\n");

    close(ss);
    for (int i = 0; i != 4; i++)
    {
        close(sr[i]);
    }
    return flag;
}

void chassis::reInitMotor()
{
    if (isMotorOK())
    {
        return;
    }
    uint8_t data[8] = {0};

    for (int i = 0; i != 4; i++)
    {
        // 速度设置为0 
        data[0] = 0x23; data[1] = 0xff; data[2] = 0x60;
        sendCan(0x601+i, data, 8);
        memset(data, 0x00, 8);
    }

    // 解决报错
    data[0] = 0x81; data[1] = 0;
    sendCan(0, data, 2);
    memset(data, 0, 8);
    
    for (int i = 0; i != 4; i++)
    {
        // 断开使能
        data[0] = 0x2b; data[1] = 0x40; data[2] = 0x60;
        sendCan(0x601+i, data, 8);
        memset(data, 0x00, 8);
    }
    // sleep(1);

    data[0] = 0x01;
    sendCan(0, data, 2);
    memset(data, 0x00, 8);

    for (int i = 0; i != 4; i++)
    {
        // 将电机配置为速度模式
        data[0] = 0x2f; data[1] = data[2] = 0x60; data[4] = 0x03;
        sendCan(0x601+i, data, 8);
        memset(data, 0x00, 8);

        // 使能电机驱动
        data[0] = 0x2b; data[1] = 0x40; data[2] = 0x60; data[4] = 0x0f;
        sendCan(0x601+i, data, 8);
        memset(data, 0x00, 8);
    }
    send2Motor(wheelLastTime);
}

void chassis::twistCb(const geometry_msgs::Twist::ConstPtr& cmd_vel)
{
    double model[2] = {0};
    model[0] = cmd_vel->linear.x;
    model[1] = cmd_vel->angular.z;
    if (teleProtocal.autoOrManual)
    {
        send2base(model);
    }
}

void chassis::send2base(double* model)
{
    ki->model2wheel(model);
    send2Motor(ki->wheel);
}

void chassis::send2Motor(double* wheel)
{
    // wheel[0]左侧前轮转速; 1右侧前轮转速; 2右侧后轮转速; 3左侧后轮转速 (rad/s)

    uint8_t data[8] = {0}, temp[4] = {0}, fixerr[8]={0};
    int32_t rotateCode;
    int shunxu[4] = {2, 1, 3, 0};
    memcpy((void*)wheelLastTime, (void*)wheel, 4*sizeof(double));
    for (int i = 0; i != 4; i++)
    {
        data[0] = 0x23; data[1] = 0xff, data[2] = 0x60;
        rotateCode = int32_t(wheel[shunxu[i]] * RR * 1e5 / (2.0 * pi));
        memcpy((void*)&data[4], (void*)&rotateCode, 4);
        // for (int j = 4; j != 0; j--)
        // {
        //     data[3+j] = temp[4-j];
        // }
        sendCan(0x601+shunxu[i], data, 8);
    }
}

void chassis::pubOdomtry()
{
    if (getFromMotor(ki->wheel) != 0)
    {
        return;
    }
    ki->wheel2model(ki->wheel);
    current_time = ros::Time::now();
    current_time_s = current_time.toSec();

    current_time = ros::Time::now();
    current_time_s = current_time.toSec();
    dt = current_time_s - last_time_s;
    vx = ki->model[0];
    vth = ki->model[1];
    x += (vx * cos(th))*dt;
    y += (vx * sin(th))*dt;
    th += vth * dt;
    last_time_s = current_time_s;

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);
    transform.transform.rotation = odom_quat;
    transform.header.stamp = current_time;
    transform.header.frame_id = "odom";
    transform.child_frame_id = "base_footprint";
    transform.transform.translation.x = x; transform.transform.translation.y = y; transform.transform.translation.z = 0;
    if (debug_alone)
    {
        br.sendTransform(transform);
    }

    odom.child_frame_id = "base_footprint";
    odom.header.frame_id = "odom";
    odom.header.seq;
    odom.header.stamp = current_time;
    odom.pose.covariance;
    odom.pose.pose.orientation = odom_quat;
    odom.pose.pose.position.x = x; odom.pose.pose.position.y = y; odom.pose.pose.position.z = 0;
    odom.twist.covariance;
    odom.twist.twist.angular.x = 0; odom.twist.twist.angular.y = 0; odom.twist.twist.angular.z = vth;
    odom.twist.twist.linear.x = vx; odom.twist.twist.linear.y = 0; odom.twist.twist.linear.z = 0;
    pubOdom.publish(odom);
}

int chassis::getFromMotor(double * wheel)
{
    int ss, nbytes, nbytes2, sr[4];    
    struct sockaddr_can addr;    
    struct ifreq ifr;    
    struct can_frame frameQ, frame2;    
    struct can_filter rfilter2[1];

    ss = socket(PF_CAN, SOCK_RAW, CAN_RAW);//创建套接字    
    strcpy(ifr.ifr_name, this->canPort.c_str());    
    ioctl(ss, SIOCGIFINDEX, &ifr);//指定can0设备    
    addr.can_family = AF_CAN;    
    addr.can_ifindex = ifr.ifr_ifindex;    
    bind(ss,(struct sockaddr*)&addr, sizeof(addr));//将套接字与can0绑定   
    setsockopt(ss, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0); 

    for (int i = 0; i != 4; i++)
    {
        sr[i] = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        ioctl(sr[i], SIOCGIFINDEX, &ifr);
        bind(sr[i], (struct sockaddr*)&addr, sizeof(addr));
        rfilter2[0].can_id = 0x581 + i;
        rfilter2[0].can_mask = CAN_SFF_MASK;
        setsockopt(sr[i], SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter2, sizeof(rfilter2));
    }
    
    for (int i = 0; i != 4; i++)
    {
        memset((void*)&frameQ.data, 0x00, sizeof(frameQ.data));
        frameQ.can_id = 0x601 + i;
        frameQ.can_dlc = 8;
        frameQ.data[0] = 0x40; frameQ.data[1] = 0x69; frameQ.data[2] = 0x60;
        memset((void*)&frame2, 0, sizeof(frame2));
        nbytes = write(ss, &frameQ, sizeof(frameQ));
        if(nbytes != sizeof(frameQ))        
        {            
            printf("Send Error frame[0]\n!");            
            break;//发送错误，退出        
        }    
        usleep(1000);
        nbytes2 = read(sr[i], &frame2, sizeof(frame2));
        if(frame2.can_dlc == 8 && frame2.can_id == (0x581+i))
        {            
            // printf("ID=0x%XDLC=%ddata[1]=0x%X\n",frame2.can_id, frame2.can_dlc,frame2.data[1]);
            // printf("[%03x] ", frame2.can_id);
            // for (int j = 0; j != 8; j++) 
            // {
            //     printf("0x%02x ", frame2.data[j]);
            // }
            // printf("\r\n");
            rotateCode2double(frame2.data, &(wheel[i]));
        } 
    }

    // for(int i = 0; i != 4; i++)
    // {
    //     printf("wheel[%d]: %lf  ", i, wheel[i]);
    // }
    // printf("\r\n");

    close(ss);
    for (int i = 0; i != 4; i++)
    {
        close(sr[i]);
    }
    return 0;
}

int chassis::rotateCode2double(u_int8_t* data, double* wheel)
{
    int rotateCode;
    if (data[0] == 0x43 && data[1] == 0x69 && data[2] == 0x60 && data[3] == 0)
    {
        memcpy((void*)&rotateCode, (void*)&(data[4]), 4);
        *wheel = (double)rotateCode * 2. * pi / (1e5 * RR);
        return 0;
    }
    return -1;
}

void chassis::teleReceive()
{
    size_t n = sp.available();
    double xyTheta[3] = {0};
    memset((void*)uartBuffer, 0, sizeof(uartBuffer));
    if (n != 0)
    {
        if (n >= bufferLen)
        {
            ROS_INFO("too many data!");
            return;
        }
        n = sp.read(uartBuffer, n);

        // for (int i = 0; i < n; i++)
        // {
        //     printf("0x%02x ", uartBuffer[i]);
        // }
        // printf("\r\n");
        
        if (teleProtocal.CRC_cal(uartBuffer, n))
        {
            teleProtocal.parse_buffer(uartBuffer, n);
            switch (teleProtocal.functionCode)
            {
            case 0x01:
                if (!debug_alone)
                {
                    pathFollowingAction_ -> cancelGoal();
                }
                send2base(teleProtocal.model);
                break;
            case 0x02:
                xyTheta[2] = teleProtocal.initAngle;
                setOdom(xyTheta);
                goalPathFollowing.initAngle = teleProtocal.initAngle;
                goalPathFollowing.longitudes.assign(teleProtocal.longitudes.begin(), teleProtocal.longitudes.end());
                goalPathFollowing.latitudes.assign(teleProtocal.latitudes.begin(), teleProtocal.latitudes.end());
                if (!debug_alone)
                {
                    pathFollowingAction_ -> sendGoal(goalPathFollowing,
                                                    boost::bind(&chassis::pathFollowingDCB, this, _1, _2), 
                                                    boost::bind(&chassis::pathFollowingACB, this),
                                                    boost::bind(&chassis::pathFollowingFCB, this, _1));
                }
                break;
            case 0x03:
                break;
            default:
                break;
            }
        }
    }
}

void chassis::setOdom(double* xyTheta)
{
    x = xyTheta[0]; // should be 0
    y = xyTheta[1]; // should be 0
    th = xyTheta[2]; // 车辆初始时刻朝向，东偏北多少弧度
}

// doneCB
void chassis::pathFollowingDCB(const actionlib::SimpleClientGoalState& state, const chassis_navigation::pathFollowingResultConstPtr& result)
{
    ROS_INFO("finished!");
}

// activeCB
void chassis::pathFollowingACB()
{
    ROS_INFO("active!");
}

// feedbackCB
void chassis::pathFollowingFCB(const chassis_navigation::pathFollowingFeedbackConstPtr& feedback)
{
    ROS_INFO("progress_bar:[%lf]", feedback->progress_bar);
}