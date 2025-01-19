#ifndef STANDING_H
#define STANDING_H

#include "FSMState.h"
#include "../../ConvexMPC/ConvexMPCLocomotion.h"
// #include "../../include/common/SwingLegController.h"
#include "../../include/common/StandLegController.h"


class FSMState_Standing: public FSMState
{
    public:
        FSMState_Standing(ControlFSMData *data);
        ~FSMState_Standing(){}
        void enter();
        void run();
        void exit();
        FSMStateName checkTransition();
    
    private:
        standLegController stand;
        int counter;
        Vec3<double> v_des_body;
        double turn_rate = 0;
        double pitch, roll;
        bool first;
};

#endif