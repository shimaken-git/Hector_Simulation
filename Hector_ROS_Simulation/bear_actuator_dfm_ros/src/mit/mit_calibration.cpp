
#include <iostream>
#include <vector>
#include <mit/mit.hpp>

int main(int argc, char *argv[])
{
    mit::MIT gimDriver;

    if(gimDriver.connect()){
        std::cout << "connect success." << std::endl;
    }else{
        std::cout << "connect fault." << std::endl;
        return -1;
    }

    std::vector<uint16_t> ids = {1,2};
    for(auto id:ids){
        if(gimDriver.ping(id)){
            std::cout << "id: " << id << " found." << std::endl;
            gimDriver.EntryActuator(id);
            gimDriver.SetKpKd(id, 0, 0.5);
            std::cout << "gear ratio : " << gimDriver.gear_ratio << std::endl;
            std::cout << "torque constant : " << gimDriver.torque_constant << std::endl;
        }else{
            std::cout << "id: " << id << " search fault." << std::endl; 
        }
    }
    std::map<int16_t, float> rps;
    rps[ids[0]] = 2.0;
    rps[ids[1]] = -2.0;
    std::map<int16_t, float> lmt;
    lmt[ids[0]] = 1.5;
    lmt[ids[1]] = -1.5;
    for(auto id:ids){
        bool loop = true;
        uint8_t err;
        int32_t result;
        gimDriver.On(id, err);
        usleep(100000);
        printf("id %d set velocity %f\r\n", id, rps[id]);
        gimDriver.SetVelocity(id, rps[id], 10, err);
        usleep(200000);
        while(loop){
            if(gimDriver.GetVelocity(id,result) == 0) loop = false;
            printf("velocity : %f\r\n", gimDriver.present_velocity[id]);
        }
        float limit = gimDriver.GetPosition(id, result);
        gimDriver.SetVelocity(id, 0, 10, err);
        float targetPos = limit - lmt[id];
        printf("id[%d] present pos %f targetpos %f \r\n", id, limit, targetPos);
        gimDriver.SetKpKd(id, 2.0, 0.5);
        gimDriver.SetPosition(id, targetPos, 0, err);
        sleep(1);
        gimDriver.Off(id, err);
        gimDriver.SetZeroPosition(id, err);
        gimDriver.On(id, err);
        usleep(100000);
        gimDriver.SetPosition(id, 0, 10, err);
        sleep(1);
        gimDriver.Off(id, err);
    }
   gimDriver.can_close();
}