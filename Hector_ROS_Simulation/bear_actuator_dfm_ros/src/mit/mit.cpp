#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <cstdint>
#include <iostream>

#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <math.h>
#include <mit/mit.hpp>

#define CAN_NAME "can0"

union a{
    float f;
    unsigned int i;
    unsigned char c[4];
};

namespace mit{
MIT::MIT():can_name(CAN_NAME), torque_constant(0.066), gear_ratio(10)
{

}

void MIT::SetCanDevice(std::string can_name_)
{
    can_name = can_name_;
}

void MIT::EntryActuator(uint8_t id)
{
    ids.push_back(id);
    zeropos.push_back(0);
    torque_status[id] = false;
}

void MIT::EntryZeropos(uint8_t id, float pos)
{
    for(int i = 0; i < ids.size(); i++){
        if(ids[i] == id) zeropos[i] = pos;
        break;
    }
}

int32_t MIT::connect()
{
    if((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0){
        perror("socket");
        return -2;
    }
    memset(&ifr.ifr_name, 0, sizeof(ifr.ifr_name));
    strncpy(ifr.ifr_name, can_name.c_str(), sizeof(ifr.ifr_name));

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

void MIT::can_close()
{
    close(s);
}

int32_t MIT::mit_write(uint16_t id, uint8_t *data)
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

int32_t MIT::mit_read(uint16_t *id, uint8_t *data, uint8_t *dlc)
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

bool MIT::ping(uint16_t id)
{
    int8_t rdata;
    int32_t result;
    result = GetError(id, rdata);
    if(rdata != id) return false;
    return true;
}

void MIT::decode_data(uint8_t id, uint8_t *data)
{
    uint16_t pos = data[1] * 256 + data[2];
    uint16_t spd = data[3] * 16 + (data[4] >> 4);
    uint16_t trq = (data[4] & 0x0f) * 256 + data[5];
    present_position[id] = pos * 25.0 / 65535.0 - 12.5;
    present_velocity[id] = (spd + 1) * 130.0 / 4096.0 - 65.0;
    present_torque[id] = (trq + 1) * (100.0 / 4096.0 - 50.0) * torque_constant * gear_ratio;
}

uint8_t MIT::GetError(uint16_t id, int8_t &rdata)
{
    uint16_t rid;
    uint8_t dlc;
    int32_t result;

    uint8_t _data[] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfb };
    result = mit_write(id, _data);
    if(result != 1) return result;
    result = mit_read(&rid, data, &dlc);
    rdata = data[0];
    return data[1];
}

int32_t MIT::SetZeroPosition(uint16_t id, uint8_t &err)
{
    uint16_t rid;
    uint8_t dlc;
    int32_t result;

    uint8_t _data[] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe };
    result = mit_write(id, _data);
    if(result != 1) return result;
    result = mit_read(&rid, data, &dlc);
    decode_data(id, data);
    return result;
}

int32_t MIT::On(uint16_t id, uint8_t &err)
{
    uint16_t rid;
    uint8_t dlc;
    int32_t result;

    uint8_t _data[] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc };
    result = mit_write(id, _data);
    if(result != 1) return result;
    result = mit_read(&rid, data, &dlc);
    decode_data(id, data);
    torque_status[id] = true;
    return result;
}

int32_t MIT::Off(uint16_t id, uint8_t &err)
{
    uint16_t rid;
    uint8_t dlc;
    int32_t result;

    uint8_t _data[] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd };
    result = mit_write(id, _data);
    if(result != 1) return result;
    result = mit_read(&rid, data, &dlc);
    decode_data(id, data);
    torque_status[id] = false;
    return result;
}

int32_t MIT::SetCommand(uint16_t id, float position, float velocity, float torque, float kp, float kd, int32_t &err)
{
    uint32_t ipos, ispd, itrq, ikp, ikd;
    union a cnv;
    uint16_t rid;
    int32_t result;
    ipos = (position + 12.5) * 65535.0 / 25.0;
    ispd = (velocity + 65) * 4095.0 / 130.0;
    // itrq = (trq + 255 * torque_constant * reduce_ratio) * 4095.0 / (450.0 * torque_constant * reduce_ratio);
    itrq = (torque + 50) * 4095.0 / 100.0;
    ikp = kp * 4095.0 / 500.0;
    ikd = kd * 4095.0 / 5.0;
    cnv.i = ipos;
    data[0] = cnv.c[1];
    data[1] = cnv.c[0];
    cnv.i = ispd;
    data[2] = cnv.c[1] << 4 | cnv.c[0] >> 4;
    data[3] = cnv.c[0] << 4;
    cnv.i = ikp;
    data[3] = data[3] | (cnv.c[1] & 0x0f);
    data[4] = cnv.c[0];
    cnv.i = ikd;
    data[5] = cnv.c[1] << 4 | cnv.c[0] >> 4;
    data[6] = cnv.c[0] << 4;
    cnv.i = itrq;
    data[6] = data[6] | (cnv.c[1] & 0x0f);
    data[7] = cnv.c[0];
    result = mit_write(id, data);
    if(result != 1) return result;
    result = mit_read(&rid, data, &dlc);
    if(result == 1){
        // memcpy(&err, &data[1], sizeof(uint8_t));
        decode_data(id, data);
    }
     return result;
}

int32_t MIT::SetPosition(uint16_t id, float position, uint32_t dur, uint8_t &err)
{
    uint16_t rid;
    uint8_t dlc;
    int32_t result, result_;

    result = SetCommand(id, position, 0, 0, kp[id], kd[id], result_);
     return result;
}

int32_t MIT::SetVelocity(uint16_t id, float velocity, uint32_t dur, uint8_t &err)
{
    uint16_t rid;
    uint8_t dlc;
    int32_t result, result_;

    result = SetCommand(id, 0, velocity, 0, 0, kd[id], result_);
     return result;
}

int32_t MIT::SetTorque(uint16_t id, float torque, uint32_t dur, uint8_t &err)
{
    uint16_t rid;
    uint8_t dlc;
    int32_t result, result_;

    result = SetCommand(id, 0, 0, torque / (torque_constant * gear_ratio), 0, 0, result_);
     return result;
}

void MIT::SetKpKd(uint16_t id, float _kp, float _kd)
{
    kp[id] = _kp;
    kd[id] = _kd;
}

void MIT::GetInfo(uint16_t id, int32_t &result)
{
    uint8_t err;
    result = On(id, err);
}

float MIT::GetPosition(uint16_t id, int32_t &result)
{
    uint8_t err;
    result = On(id, err);
    return present_position[id];
}

float MIT::GetVelocity(uint16_t id, int32_t &result)
{
    uint8_t err;
    result = On(id, err);
    return present_velocity[id];
}

float MIT::GetTorque(uint16_t id, int32_t &result)
{
    uint8_t err;
    result = On(id, err);
    return present_torque[id];
}

}