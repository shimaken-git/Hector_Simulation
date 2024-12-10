
#include <iostream>
#include <vector>
#include "gim/gim.hpp"

int main(int argc, char *argv[])
{
    gim::GIM gimDriver;

    if(gimDriver.connect()){
        std::cout << "connect success." << std::endl;
    }else{
        std::cout << "connect fault." << std::endl;
        return -1;
    }

    std::vector<uint16_t> ids = {1,2};
    for(auto id:ids){
        if(gimDriver.ping(id)){
            int32_t gear_ratio;
            float torque_constant, zeropos;
            std::cout << "id: " << id << " found." << std::endl;
            gimDriver.EntryActuator(id);
            gimDriver.GetIntConfig(id, 0x11, gear_ratio);
            gimDriver.GetFloatConfig(id, 0x03, torque_constant);
            gimDriver.GetZeroPotision(id, zeropos);
            std::cout << "gear ratio : " << gear_ratio << std::endl;
            std::cout << "torque constant : " << torque_constant << std::endl;
            std::cout << "zeropos : " << zeropos << std::endl;
            gimDriver.EntryZeropos(id, zeropos);
        }else{
            std::cout << "id: " << id << " search fault." << std::endl; 
        }
    }
    std::map<int16_t, float> rpm;
    rpm[ids[0]] = 7;
    rpm[ids[1]] = -7;
    std::map<int16_t, float> lmt;
    lmt[ids[0]] = 1.39;
    lmt[ids[1]] = -1.30;
    for(auto id:ids){
        bool loop = true;
        uint8_t err;
        int32_t result;
        gimDriver.On(id, err);
        gimDriver.SetVelocity(id, rpm[id], 10, err);
        sleep(1.0);
        while(loop){
            if(gimDriver.GetVelocity(id,result) == 0) loop = false;
        }
        float limit = gimDriver.GetPosition(id, result);
        gimDriver.SetVelocity(id, 0, 10, err);
        float zeroPos = limit - lmt[id];
        printf("id[%d] limit %f zeropos %f \r\n", id, limit, zeroPos);
        gimDriver.Off(id, err);
        gimDriver.SetZeroPosition(id, zeroPos, err);
        gimDriver.EntryZeropos(id, zeroPos);
        sleep(1.0);
        gimDriver.On(id, err);
        gimDriver.SetPosition(id, 0, 500, err);
        sleep(1.0);
        gimDriver.Off(id, err);
    }
   gimDriver.gim_close();
}