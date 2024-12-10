#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <cstdint>

#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <math.h>
#include "gim/gim.hpp"

#define CAN_NAME "can0"

union a{
    float f;
    unsigned int i;
    unsigned char c[4];
};

namespace gim{
GIM::GIM():torque_constant(0.066), gear_ratio(10)
{

}

void GIM::EntryActuator(uint16_t id)
{
    ids.push_back(id);
}

void GIM::EntryZeropos(uint16_t id, float pos)
{
    zeropos[id] = pos;
}

int32_t GIM::connect()
{
    if((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0){
        perror("socket");
        return -2;
    }
    memset(&ifr.ifr_name, 0, sizeof(ifr.ifr_name));
    strncpy(ifr.ifr_name, CAN_NAME, sizeof(ifr.ifr_name));

    ifr.ifr_ifindex = if_nametoindex(ifr.ifr_name);
    if(! ifr.ifr_ifindex){
        perror("if_nametoindex");
        return -3;
    }
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    // setsockopt(p, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);

    /* no loopback */
    // loopback = 0; /* 0 = disabled, 1 = enabled(default) */
    // setsockopt(s, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopback, sizeof(loopback));

    if(bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
        perror("bind");
        return -4;
    }
    return 1;
}

void GIM::gim_close()
{
    close(s);
}

int32_t GIM::gim_write(uint16_t id, uint8_t *data)
{
    struct can_frame frame;
    int dlc = 8;

    memset(&frame, 0, sizeof(frame));
    frame.can_id = id;
    frame.can_dlc = dlc;
    memcpy(&(frame.data[0]), &(data[0]), dlc); 
    if(write(s, &frame, CAN_MTU) < CAN_MTU){
        perror("write");
        return -5;
    }
    return 1;
}

int32_t GIM::gim_read(uint16_t *id, uint8_t *data, uint8_t *dlc)
{
    fd_set rdfs;
    struct timeval timeout;
    struct can_frame frame;
    int32_t nbytes;

    while(true){
        FD_ZERO(&rdfs);
        FD_SET(s, &rdfs);

        timeout.tv_sec = 1;
        timeout.tv_usec = 0;

        int ret = select(s+1, &rdfs, NULL, NULL, &timeout);
        if(ret < 0){
            perror("select");
            return -4;
        }else if(0 == ret){
            return -1; //time out
        }else{
            break;
        }
    }

    nbytes = read(s, &frame, sizeof(frame));
    if(nbytes < 0){
        perror("recv");
        return -5;
    }

    if(nbytes == sizeof(frame)){
        *id = frame.can_id;
        *dlc = frame.can_dlc;
        memcpy(data, frame.data, CAN_MAX_DLEN);
    }else{
        fprintf(stderr, "recv size not std-frame.\n");
    }
    return 1;
}

bool GIM::ping(uint16_t id)
{
    int32_t rdata;
    int32_t result;
    result = GetIntConfig(id, 0x12, rdata);
    if(rdata != id) return false;
    return true;
}

void GIM::decode_data(uint16_t id, uint8_t *data)
{
    int32_t pos = data[4] * 256 + data[3];
    int32_t spd = data[5] * 16 + (data[6] >> 4);
    int32_t trq = (data[6] & 0x0f) * 256 + data[7];
    present_position[id] = pos * 25.0 / 65535.0 - 12.5;
    present_velocity[id] = spd * 130.0 / 4095.0 - 65.0;
    present_torque[id] = trq * (450.0 / 4095.0 - 225.0) * torque_constant * gear_ratio;
}

int32_t GIM::GetIntConfig(uint16_t id, uint8_t ConfId, int32_t &rdata)
{
    uint16_t rid;
    uint8_t dlc;
    int32_t result;

    memset(data, 0, sizeof(data));
    data[0] = 0x84;
    data[1] = 0x00;
    data[2] = ConfId;
    result = gim_write(id, data);
    if(result != 1) return result;
    result = gim_read(&rid, data, &dlc);
    if(result == 1) memcpy(&rdata, &data[4], sizeof(int32_t));
    return result;
}

int32_t GIM::GetFloatConfig(uint16_t id, uint8_t ConfId, float &rdata)
{
    uint16_t rid;
    uint8_t dlc;
    int32_t result;

    memset(data, 0, sizeof(data));
    data[0] = 0x84;
    data[1] = 0x01;
    data[2] = ConfId;
    result = gim_write(id, data);
    if(result != 1) return result;
    result = gim_read(&rid, data, &dlc);
    if(result == 1) memcpy(&rdata, &data[4], sizeof(float));
    return result;
}

int32_t GIM::GetInfo(uint16_t id, uint8_t IndId, float &rdata)
{
    uint16_t rid;
    uint8_t dlc;
    int32_t result;

    memset(data, 0, sizeof(data));
    data[0] = 0xb4;
    data[1] = IndId;
    result = gim_write(id, data);
    if(result != 1) return result;
    result = gim_read(&rid, data, &dlc);
    if(result == 1) memcpy(&rdata, &data[4], sizeof(float));
    return result;
}

int32_t GIM::GetZeroPotision(uint16_t id, float &pos)
{
    int32_t ipos;
    int32_t result = GetIntConfig(id, 0x14, ipos);
    if(result == 1){
        pos = ipos * 2.0 * M_PI / 65536.0;
    }
    return result;
}

int32_t GIM::SetIntConfig(uint16_t id, uint8_t ConfId, int32_t setdata, uint8_t &err)
{
    uint16_t rid;
    uint8_t dlc;
    int32_t result;

    memset(data, 0, sizeof(data));
    data[0] = 0x83;
    data[1] = 0x00;
    data[2] = ConfId;
    memcpy(&data[4], &setdata, sizeof(int32_t));
    result = gim_write(id, data);
    if(result != 1) return result;
    result = gim_read(&rid, data, &dlc);
    return result;
}

int32_t GIM::SetZeroPosition(uint16_t id, float zeroPos, uint8_t &err)
{
    int32_t idata = (int)(zeroPos * 65536.0 / (2 * M_PI));
    int32_t result = SetIntConfig(id, 0x14, idata, err);
    return result;
}

int32_t GIM::On(uint16_t id, uint8_t &err)
{
    uint16_t rid;
    uint8_t dlc;
    int32_t result;

    memset(data, 0, sizeof(data));
    data[0] = 0x91;
    result = gim_write(id, data);
    if(result != 1) return result;
    result = gim_read(&rid, data, &dlc);
    if(result == 1){
        memcpy(&err, &data[1], sizeof(uint8_t));
    }
     return result;
}

int32_t GIM::Off(uint16_t id, uint8_t &err)
{
    uint16_t rid;
    uint8_t dlc;
    int32_t result;

    memset(data, 0, sizeof(data));
    data[0] = 0x92;
    result = gim_write(id, data);
    if(result != 1) return result;
    result = gim_read(&rid, data, &dlc);
    if(result == 1){
        memcpy(&err, &data[1], sizeof(uint8_t));
    }
     return result;
}

int32_t GIM::SetPosition(uint16_t id, float position, uint32_t dur, uint8_t &err)
{
    uint16_t rid;
    uint8_t dlc;
    int32_t result;

    data[0] = 0x95;
    memcpy(&data[1], &position, sizeof(float));
    memcpy(&data[5], &dur, sizeof(float) - 1);
    result = gim_write(id, data);
    if(result != 1) return result;
    result = gim_read(&rid, data, &dlc);
    if(result == 1){
        memcpy(&err, &data[1], sizeof(uint8_t));
        decode_data(id, data);
    }
     return result;
}

int32_t GIM::SetVelocity(uint16_t id, float velocity, uint32_t dur, uint8_t &err)
{
    uint16_t rid;
    uint8_t dlc;
    int32_t result;

    data[0] = 0x94;
    memcpy(&data[1], &velocity, sizeof(float));
    memcpy(&data[5], &dur, sizeof(float) - 1);
    result = gim_write(id, data);
    if(result != 1) return result;
    result = gim_read(&rid, data, &dlc);
    if(result == 1){
        memcpy(&err, &data[1], sizeof(uint8_t));
        decode_data(id, data);
    }
     return result;
}

int32_t GIM::SetTorque(uint16_t id, float torque, uint32_t dur, uint8_t &err)
{
    uint16_t rid;
    uint8_t dlc;
    int32_t result;

    data[0] = 0x93;
    memcpy(&data[1], &torque, sizeof(float));
    memcpy(&data[5], &dur, sizeof(float) - 1);
    result = gim_write(id, data);
    if(result != 1) return result;
    result = gim_read(&rid, data, &dlc);
    if(result == 1){
        memcpy(&err, &data[1], sizeof(uint8_t));
        decode_data(id, data);
    }
     return result;
}

float GIM::GetPosition(uint16_t id, int32_t &result)
{
    float fdata;
    result = GetInfo(id, 0x13, fdata);
    return (fdata - zeropos[id]);
}

float GIM::GetVelocity(uint16_t id, int32_t &result)
{
    float fdata;
    result = GetInfo(id, 0x14, fdata);

    return fdata;
}

float GIM::GetTorque(uint16_t id, int32_t &result)
{
    float fdata;
    result = GetInfo(id, 0x09, fdata);
    return fdata;
}

}