#include "telecom/telecom_protocal.h"
#define pi 3.14159265
#define HeadLen 2
#define manualHeadLen 4
#define vxSF    0.039
#define wzSF    0.0156
#define angleSF 0.00549
#define longitudeSF 1./1800000.
#define latitudeSF  1./1800000.
// #define deg2rad(deg) deg/180. * (pi)
uint8_t dataHead[HeadLen] = {0xab, 0xab};
uint8_t manualHead[manualHeadLen] = {0xab, 0xab, 1, 2};

double deg2rad(double deg)
{
    while(deg >= 180.) deg -= 180.;
    while(deg < -180.) deg += 180.;
    double ret = deg/180. * (pi);
}
Telecom_Protocal::Telecom_Protocal()
{
    autoOrManual = true;
}

Telecom_Protocal::~Telecom_Protocal()
{

}

void Telecom_Protocal::parse_buffer(uint8_t* buf, int len)
{
    int i = 0;
    char vxCode = 0;
    char wzCode = 0;
    char dataLenth = 0;
    signed short initialAngleCode = 0;
    char isE = 0, isN = 0; // 0: E, 1: W; 0: N, 1: S
    int latitudeCode = 0, longitudeCode = 0;
    double longitude = 0, latitude = 0;
    for (i = 0; i != len-1; i++)
    {
        printf("0x%02x ", buf[i]);
    }
    printf("0x%02x\r\n", buf[len-1]);
    for (i = 0; i != HeadLen; i++)
    {
        if (dataHead[i] != buf[i])
        {
            return;
        }
    }
    readBytes(buf, &i, sizeof(char), (void*)&functionCode);
    switch (functionCode)
    {
    case 0x00:
        /* shutdown */
        break;
    case 0x01:
        /*manual*/
        readBytes(buf, &i, sizeof(char), (void*)&dataLenth);
        readBytes(buf, &i, sizeof(char), (void*)&vxCode);
        model[0] = (double)vxCode * vxSF;
        readBytes(buf, &i, sizeof(char), (void*)&wzCode);
        model[1] = (double)wzCode * wzSF;
        autoOrManual = false;
        break;
    case 0x02:
        /*auto*/
        readBytes(buf, &i, sizeof(char), (void*)&autoFunctionCode); // only support 0
        switch (autoFunctionCode)
        {
        case 0:
            /* 前往指定坐标 */
            readBytes(buf, &i, sizeof(char), (void*)&dataLenth);
            dataLenth -= 2; // 减去初始角度占用的字节数
            readBytes(buf, &i, sizeof(signed short), (void*)&initialAngleCode);
            initAngle = (double)initialAngleCode * angleSF;
            initAngle = deg2rad(initAngle);
            for (int j = 0; j != (dataLenth/10); j++)
            {
                readBytes(buf, &i, sizeof(char), (void*)&isE);
                readBytes(buf, &i, sizeof(int), (void*)&longitudeCode);
                longitude = (double)longitudeCode * longitudeSF;
                if (isE == 1)
                {
                    longitude *= -1.;
                }
                longitudes.push_back(longitude);
                readBytes(buf, &i, sizeof(char), (void*)&isN);
                readBytes(buf, &i, sizeof(int), (void*)&latitudeCode);
                latitude = (double)latitudeCode * longitudeSF;
                if (isN == 1)
                {
                    latitude *= -1.;
                }
                latitudes.push_back(latitude);
            }
            break;
        default:
            break;
        }
        autoOrManual = true;
        break;
    case 0x03:
        /* reinit motor */
        break;
    default:
        break;
    }
}

bool Telecom_Protocal::CRC_cal(uint8_t* buf, int len)
{
    return true;
}

void Telecom_Protocal::readBytes(uint8_t* buf, int* i, int byte_num, void* Dest)
{
    memcpy(Dest, (void*)&(buf[*i]), byte_num);
    *i += byte_num;
}