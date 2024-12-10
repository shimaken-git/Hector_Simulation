#ifndef GIM_INCLUDE_H_
#define GIM_INCLUDE_H_

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <cstdint>
#include <vector>
#include <map>

#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <math.h>

namespace gim{
class GIM{
    public:
        GIM();
        ~GIM(){};

        void EntryActuator(uint16_t id);
        void EntryZeropos(uint16_t id, float zeropos);
        int32_t connect();
        void gim_close();
        int32_t gim_write(uint16_t id, uint8_t *data);
        int32_t gim_read(uint16_t *id, uint8_t *data, uint8_t *dlc);
        bool ping(uint16_t id);
        void decode_data(uint16_t id, uint8_t *data);
        int32_t GetIntConfig(uint16_t id, uint8_t ConfId, int32_t &rdata);
        int32_t GetFloatConfig(uint16_t id, uint8_t ConfId, float &rdata);
        int32_t GetZeroPotision(uint16_t id, float &pos);
        int32_t GetInfo(uint16_t id, uint8_t IndId, float &rdata);
        int32_t SetIntConfig(uint16_t id, uint8_t ConfId, int32_t data, uint8_t &err);
        int32_t SetZeroPosition(uint16_t id, float SetZeroPosition, uint8_t &err);
        int32_t On(uint16_t id, uint8_t &err);
        int32_t Off(uint16_t id, uint8_t &err);
        int32_t SetPosition(uint16_t id, float position, uint32_t dur, uint8_t &err);
        int32_t SetVelocity(uint16_t id, float velocity, uint32_t dur, uint8_t &err);
        int32_t SetTorque(uint16_t id, float torque, uint32_t dur, uint8_t &err);
        float GetPosition(uint16_t id, int32_t &result);
        float GetVelocity(uint16_t id, int32_t &result);
        float GetTorque(uint16_t id, int32_t &result);

    private:
        float torque_constant;
        float gear_ratio;
        int32_t s;
        struct ifreq ifr;
        struct sockaddr_can addr;
        // struct can_frame frame;
        int32_t loopback;

        uint8_t data[CAN_MAX_DLEN];
        uint8_t dlc;
        std::vector<uint16_t> ids;
        std::map<uint16_t, float> zeropos;

    public:
        std::map<uint8_t, float> present_position;
        std::map<uint8_t, float> present_velocity;
        std::map<uint8_t, float> present_torque;

};
}

#endif