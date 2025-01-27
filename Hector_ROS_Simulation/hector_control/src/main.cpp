#include <iostream>
#include <unistd.h>
#include <csignal>
#include <sched.h>
#include <string>
#include <thread>

// #define BEAR_REAL

#include "../include/common/ControlFSMData.h"
#include "../include/common/OrientationEstimator.h"
#include "../include/common/PositionVelocityEstimator.h"
#ifdef BEAR_REAL
#include "../include/interface/BearIO.h"
#else
#include "../include/interface/CheatIO.h"
#endif
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
    ros::Publisher gpworld_pub, pworld_pub;
    ros::Publisher gvworld_pub, vworld_pub;
    IOInterface *ioInter;
    ros::init(argc, argv, "hector_control", ros::init_options::AnonymousName);
    ros::NodeHandle nh;
    gpworld_pub = nh.advertise<geometry_msgs::Vector3>("/gpworld", 1);
    pworld_pub = nh.advertise<geometry_msgs::Vector3>("/pworld", 1);
    gvworld_pub = nh.advertise<geometry_msgs::Vector3>("/gvworld", 1);
    vworld_pub = nh.advertise<geometry_msgs::Vector3>("/vworld", 1);
    

    double dt = 0.001;
    Biped biped;
    // biped.setBiped();

#ifdef BEAR_REAL
    std::string robot_name = "lambad_leg";
    ioInter = new BearIO(robot_name, biped.height);
#else
    std::string robot_name = "hector";
    ioInter = new CheatIO(robot_name, biped.height);
#endif
    std::cout << "robot name " << robot_name << std::endl;
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
    
    geometry_msgs::Vector3 gpworld, pworld, gvworld, vworld;
    while(running)
    {
        ioInter->sendRecv(cmd, state);
        auto result = _controlData->_stateEstimator->getResult();
        gpworld.x = result.position[0];
        gpworld.y = result.position[1];
        gpworld.z = result.position[2];
        pworld.x = result.p_world[0];
        pworld.y = result.p_world[1];
        pworld.z = result.p_world[2];
        gvworld.x = result.vWorld[0];
        gvworld.y = result.vWorld[1];
        gvworld.z = result.vWorld[2];
        vworld.x = result.v_world[0];
        vworld.y = result.v_world[1];
        vworld.z = result.v_world[2];
        gpworld_pub.publish(gpworld);
        pworld_pub.publish(pworld);
        gvworld_pub.publish(gvworld);
        vworld_pub.publish(vworld);
        rate.sleep();
    }
    
    delete _controlData;
    return 0;

}
