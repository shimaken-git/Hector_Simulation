#include <iostream>
#include <unistd.h>
#include <csignal>
#include <sched.h>
#include <string>
#include <thread>

#include "../include/common/ControlFSMData.h"
#include "../include/common/OrientationEstimator.h"
#include "../include/common/PositionVelocityEstimator.h"
#include "../include/interface/CheatIO.h"
#include "../include/FSM/FSM.h"

#include <geometry_msgs/Vector3.h>

bool running = true;

void ShutDown(int sig)
{
    std::cout << "stop" << std::endl;
    running = false;
}

void runFSMController(FSM* _FSMController)
{
    ros::Rate rate(1000); 
    while (running)
    {
        _FSMController->run();
        rate.sleep();
    }
}

int main(int argc, char ** argv)
{
    ros::Publisher wpos_pub, gpos_pub;
    IOInterface *ioInter;
    ros::init(argc, argv, "hector_control", ros::init_options::AnonymousName);
    ros::NodeHandle nh;
    wpos_pub = nh.advertise<geometry_msgs::Vector3>("/wpos", 1);
    gpos_pub = nh.advertise<geometry_msgs::Vector3>("/gpos", 1);
    
    std::string robot_name = "hector";
    std::cout << "robot name " << robot_name << std::endl;

    double dt = 0.001;
    Biped biped;
    // biped.setBiped();

#ifdef BEAR_REAL
    ioInter = new BearIO(robot_name, biped.height);
#else
    ioInter = new CheatIO(robot_name, biped.height);
#endif
    ros::Rate rate(1000);

    LegController* legController = new LegController(biped);
    LowlevelCmd* cmd = new LowlevelCmd();
    LowlevelState* state = new LowlevelState();

    std::cout << "start setup " << std::endl;
    StateEstimate stateEstimate;
    StateEstimatorContainer* stateEstimator = new StateEstimatorContainer(state,
                                                                          legController->data,
                                                                          &stateEstimate);

    stateEstimator->addEstimator<CheaterOrientationEstimator>();   
    stateEstimator->addEstimator<CheaterPositionVelocityEstimator>();   

    std::cout << "setup state etimator" << std::endl;                                                             

    DesiredStateCommand* desiredStateCommand = new DesiredStateCommand(&stateEstimate, dt);

    ControlFSMData* _controlData = new ControlFSMData;
    _controlData->_biped = &biped;
    _controlData->_stateEstimator = stateEstimator;
    _controlData->_legController = legController;
    _controlData->_desiredStateCommand = desiredStateCommand;
    _controlData->_interface = ioInter;
    _controlData->_lowCmd = cmd;
    _controlData->_lowState = state;

    std::cout << "FSMController construct start" << std::endl;
    FSM* _FSMController = new FSM(_controlData);
    std::cout << "FSMController construct end" << std::endl;

    // FSM in a separate thread
    std::thread fsm_thread(runFSMController, _FSMController);    

    signal(SIGINT, ShutDown);
    
    geometry_msgs::Vector3 wpos, gpos;
    while(running)
    {
        ioInter->sendRecv(cmd, state);
        auto result = _controlData->_stateEstimator->getResult();
        wpos.x = result.position[0];
        wpos.y = result.position[1];
        wpos.z = result.position[2];
        gpos.x = result.p_world[0];
        gpos.y = result.p_world[1];
        gpos.z = result.p_world[2];
        wpos_pub.publish(wpos);
        gpos_pub.publish(gpos);
        rate.sleep();
    }
    
    delete _controlData;
    return 0;

}
