#ifndef MIT_INCLUDE_H_
#define MIT_INCLUDE_H_

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

namespace mit{
class MIT{
    public:
        MIT();
        ~MIT(){};

        void SetCanDevice(std::string can_name_);
        void EntryActuator(uint8_t id);
        void EntryZeropos(uint8_t id, float zeropos);
        int32_t connect();
        void mit_close();
        int32_t mit_write(uint16_t id, uint8_t *data);
        int32_t mit_read(uint16_t *id, uint8_t *data, uint8_t *dlc);
        bool ping(uint16_t id);
        void decode_data(uint8_t id, uint8_t *data);
        uint8_t GetError(uint16_t id, int8_t &rdata);
        int32_t SetZeroPosition(uint16_t id, uint8_t &err);
        int32_t On(uint16_t id, uint8_t &err);
        int32_t Off(uint16_t id, uint8_t &err);
        int32_t SetCommand(uint16_t id, float position, float velocity, float torque, float kp, float kd);
        int32_t SetPosition(uint16_t id, float position, uint8_t &err);
        int32_t SetVelocity(uint16_t id, float velocity, uint8_t &err);
        int32_t SetTorque(uint16_t id, float torque, uint8_t &err);
        void SetParam(uint16_t id, float _kp, float _kd);
        void GetInfo(uint16_t id, int32_t &result);
        float GetPosition(uint16_t id, int32_t &result);
        float GetVelocity(uint16_t id, int32_t &result);
        float GetTorque(uint16_t id, int32_t &result);

    private:
        std::string can_name;
        float torque_constant;
        float gear_ratio;
        int32_t s;
        struct ifreq ifr;
        struct sockaddr_can addr;
        // struct can_frame frame;
        int32_t loopback;

        uint8_t data[CAN_MAX_DLEN];
        uint8_t dlc;
        std::vector<float> zeropos;

    public:
        std::vector<uint8_t> ids;
        std::map<uint8_t, float> present_position;
        std::map<uint8_t, float> present_velocity;
        std::map<uint8_t, float> present_torque;
        std::map<uint8_t, float> kp;
        std::map<uint8_t, float> kd;
        std::map<uint8_t, float> torque_offset;

};
}

#endif