#include "gnss_driver/gnss_driver.h"

GNSS_Driver::GNSS_Driver(): nh("~")
{
    nh.param<std::string>("gnss_port", telePort, "/dev/ttyS2");
    current_time = ros::Time::now();
    initUart();
    pubGnrmc = nh.advertise<gnss_driver::GNRMC>("/gnrmc", 10);
    pubGngga = nh.advertise<gnss_driver::GNGGA>("/gngga", 10);
}

GNSS_Driver::~GNSS_Driver()
{
    sp.close();
}

void GNSS_Driver::initUart()
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
        ROS_ERROR_STREAM("Unable to open gnss port!");
    }
    if (!sp.isOpen())
    {
        ROS_ERROR_STREAM("Unable to open gnss port!");
    }
}

void GNSS_Driver::gnssReceive()
{
    size_t n = sp.available();
    int ret = 0;
    bool isRMC = true;
    memset((void*)uartBuffer, 0, sizeof(uartBuffer));
    if (n != 0)
    {
        if (n > sizeof(uartBuffer))
        {
            n = sp.read(uartBuffer, sizeof(uartBuffer));
        }
        else
        {
            n = sp.read(uartBuffer, n);
        }
        ret = parser((char*)uartBuffer, &isRMC);
        if (ret != -1)
        {
            if (isRMC)
            {
                pubGnrmc.publish(gnrmc);
            }
            else
            {
                pubGngga.publish(gngga);
            }
        }
    }
}

int GNSS_Driver::parser(char *data, bool * isRMC)
{
    char* start;
    char *field;
    int index = 0;
    start = strstr(data,"GNRMC");
    if (start)
    {
        *isRMC = true;
        field = strtok(start,",");
        while (field)
        {
            index++;
            field = strtok(NULL,",");
            if (index == 3)
            {
                gnrmc.longitude = atof(field);
                gnrmc.longitude = (int)(gnrmc.longitude / 100) + fmod(gnrmc.longitude, 100) / 60.0;
            }
            if (index == 4)
            {
                if(strcmp(field, "S") == 0)
                {
                    gnrmc.longitude *= -1.;
                }
            }
            if (index == 5)
            {
                gnrmc.latitude = atof(field);
                gnrmc.latitude = (int)(gnrmc.latitude / 100) + fmod(gnrmc.latitude, 100) / 60.0;
            }
            if (index == 6)
            {
                if(strcmp(field, "W") == 0)
                {
                    gnrmc.latitude *= -1.;
                }
            }
            if (index == 7)
            {
                gnrmc.speed = atof(field);
                gnrmc.speed *= 0.5144444;
            }
            if (index == 8)
            {
                gnrmc.yaw = atof(field);
                break;
            }
        }
        return 0;
    }
    else
    {
        start = strstr(data,"GNGGA");
        if (start)
        { 
            *isRMC = false;
            field = strtok(start,",");
            while (field)
            {
                index++;
                field = strtok(NULL,",");
                if (index == 2)
                {
                    gngga.longitude = atof(field);
                    gngga.longitude = (int)(gngga.longitude / 100) + fmod(gngga.longitude, 100) / 60.0;
                }
                if (index == 3)
                {
                    if(strcmp(field, "S") == 0)
                    {
                        gngga.longitude *= -1.;
                    }
                }
                if (index == 4)
                {
                    gngga.latitude = atof(field);
                    gngga.latitude = (int)(gngga.latitude / 100) + fmod(gngga.latitude, 100) / 60.0;
                }
                if (index == 5)
                {
                    if(strcmp(field, "W") == 0)
                    {
                        gngga.latitude *= -1.;
                    }
                    break;
                }
            }
        }
        return 0;
    }

    return -1;
}