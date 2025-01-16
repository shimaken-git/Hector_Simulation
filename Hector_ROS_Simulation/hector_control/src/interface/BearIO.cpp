#include "../../include/interface/BearIO.h"
#include "../../include/interface/KeyBoard.h"
#include <iostream>
#include <unistd.h>
#include <csignal>
#include "../../include/common/robot_select.h"

inline void RosShutDown(int sig){
    ROS_INFO("ROS interface shutting down!");
	ros::shutdown();
}

BearIO::BearIO(std::string robot_name, double _height):IOInterface(), _subSpinner(1), height(_height)
{
    std::cout << "The control interface for ROS Gazebo simulation with cheat states from gazebo" << std::endl;
    _robot_name = robot_name;

    // start subscriber
    initRecv();
    _subSpinner.start();
    usleep(3000);     //wait for subscribers start
    // initialize publisher
    initSend();   

    signal(SIGINT, RosShutDown);

    cmdPanel = new KeyBoard();
}

BearIO::~BearIO()
{
    ros::shutdown();
}

void BearIO::sendRecv(const LowlevelCmd *cmd, LowlevelState *state)
{
    sendCmd(cmd);
    recvState(state);
    cmdPanel->updateVelCmd(state);

    state->userCmd = cmdPanel->getUserCmd();
    state->userValue = cmdPanel->getUserValue();
}

void BearIO::sendCmd(const LowlevelCmd *cmd)
{
    for(int i = 0; i < 10; i++){
        _lowCmd.motorCmd[i].mode = 0X0A; // alwasy set it to 0X0A
        _lowCmd.motorCmd[i].q = cmd->motorCmd[i].q;
        _lowCmd.motorCmd[i].dq = cmd->motorCmd[i].dq;
        _lowCmd.motorCmd[i].tau = cmd->motorCmd[i].tau;
        _lowCmd.motorCmd[i].Kd = cmd->motorCmd[i].Kd;
        _lowCmd.motorCmd[i].Kp = cmd->motorCmd[i].Kp;
    }
    for(int m = 0; m < 10; m++){
        _servo_pub[m].publish(_lowCmd.motorCmd[m]);
    }

}

void BearIO::recvState(LowlevelState *state)
{
    for(int i = 0; i < 10; i++)
    {
        state->motorState[i].q = _highState.motorState[i].q;
        state->motorState[i].dq = _highState.motorState[i].dq;
        state->motorState[i].tauEst = _highState.motorState[i].tauEst;
    }
    for(int i = 0; i < 3; i++){
        state->imu.quaternion[i] = _highState.imu.quaternion[i];
        state->imu.gyroscope[i] = _highState.imu.gyroscope[i];
        state->position[i] = _highState.position[i];
        state->vWorld[i] = _highState.velocity[i];
    }
    if(state->position[2] == 0) state->position[2] = height;    //初回にゼロを返してしまうので固定値を入れる。
    state->imu.quaternion[3] = _highState.imu.quaternion[3];
}

void BearIO::initSend(){
    _servo_pub[0] = _nm.advertise<westwood_legged_msgs::MotorCmd>( "/" + _robot_name + "/L_hip_controller/command", 1);
    _servo_pub[1] = _nm.advertise<westwood_legged_msgs::MotorCmd>( "/" + _robot_name + "/L_hip2_controller/command", 1);
    _servo_pub[2] = _nm.advertise<westwood_legged_msgs::MotorCmd>( "/" + _robot_name + "/L_thigh_controller/command", 1);
    _servo_pub[3] = _nm.advertise<westwood_legged_msgs::MotorCmd>( "/" + _robot_name + "/L_calf_controller/command", 1);
    _servo_pub[4] = _nm.advertise<westwood_legged_msgs::MotorCmd>( "/" + _robot_name + "/L_toe_controller/command", 1);
    _servo_pub[5] = _nm.advertise<westwood_legged_msgs::MotorCmd>( "/" + _robot_name + "/R_hip_controller/command", 1);
    _servo_pub[6] = _nm.advertise<westwood_legged_msgs::MotorCmd>( "/" + _robot_name + "/R_hip2_controller/command", 1);
    _servo_pub[7] = _nm.advertise<westwood_legged_msgs::MotorCmd>( "/" + _robot_name + "/R_thigh_controller/command", 1);
    _servo_pub[8] = _nm.advertise<westwood_legged_msgs::MotorCmd>( "/" + _robot_name + "/R_calf_controller/command", 1);
    _servo_pub[9] = _nm.advertise<westwood_legged_msgs::MotorCmd>( "/" + _robot_name + "/R_toe_controller/command", 1);
}

void BearIO::initRecv(){
    _state_sub = _nm.subscribe("/gazebo/model_states", 1, &BearIO::StateCallback, this);
    _servo_sub[0] = _nm.subscribe( "/" + _robot_name + "/L_hip_controller/state", 1, &BearIO::LhipCallback, this);
    _servo_sub[1] = _nm.subscribe( "/" + _robot_name + "/L_hip2_controller/state", 1, &BearIO::Lhip2Callback, this);
    _servo_sub[2] = _nm.subscribe( "/" + _robot_name + "/L_thigh_controller/state", 1, &BearIO::LthighCallback, this);
    _servo_sub[3] = _nm.subscribe( "/" + _robot_name + "/L_calf_controller/state", 1, &BearIO::LcalfCallback, this);
    _servo_sub[4] = _nm.subscribe( "/" + _robot_name + "/L_toe_controller/state", 1, &BearIO::LtoeCallback, this);
    _servo_sub[5] = _nm.subscribe( "/" + _robot_name + "/R_hip_controller/state", 1, &BearIO::RhipCallback, this);
    _servo_sub[6] = _nm.subscribe( "/" + _robot_name + "/R_hip2_controller/state", 1, &BearIO::Rhip2Callback, this);
    _servo_sub[7] = _nm.subscribe( "/" + _robot_name + "/R_thigh_controller/state", 1, &BearIO::RthighCallback, this);
    _servo_sub[8] = _nm.subscribe( "/" + _robot_name + "/R_calf_controller/state", 1, &BearIO::RcalfCallback, this);
    _servo_sub[9] = _nm.subscribe( "/" + _robot_name + "/R_toe_controller/state", 1, &BearIO::RtoeCallback, this);

    // _contact_sub[0] = _nm.subscribe("/bumper_LF", 1, &BearIO::ContactLFCallback, this);
    // _contact_sub[1] = _nm.subscribe("/bumper_LB", 1, &BearIO::ContactLBCallback, this);
    // _contact_sub[2] = _nm.subscribe("/bumper_RF", 1, &BearIO::ContactRFCallback, this);
    // _contact_sub[3] = _nm.subscribe("/bumper_RB", 1, &BearIO::ContactRBCallback, this);
    _contact_sub = _nm.subscribe("/touch", 1, &BearIO::ContactCallback, this);

    // _contact_pub[0] = _nm.advertise<std_msgs::UInt8>( "/bumper_LF_int", 1);
    // _contact_pub[1] = _nm.advertise<std_msgs::UInt8>( "/bumper_LB_int", 1);
    // _contact_pub[2] = _nm.advertise<std_msgs::UInt8>( "/bumper_RF_int", 1);
    // _contact_pub[3] = _nm.advertise<std_msgs::UInt8>( "/bumper_RB_int", 1);
}

void BearIO::StateCallback(const gazebo_msgs::ModelStates & msg)
{
    int robot_index;
    // std::cout << msg.name.size() << std::endl;
    for(int i = 0; i < msg.name.size(); i++)
    {
        if(msg.name[i] == _robot_name)
        {
            robot_index = i;
        }
    }

    _highState.position[0] = msg.pose[robot_index].position.x;
    _highState.position[1] = msg.pose[robot_index].position.y;
    _highState.position[2] = msg.pose[robot_index].position.z;

    _highState.velocity[0] = msg.twist[robot_index].linear.x;
    _highState.velocity[1] = msg.twist[robot_index].linear.y;
    _highState.velocity[2] = msg.twist[robot_index].linear.z;

    _highState.imu.quaternion[0] = msg.pose[robot_index].orientation.w;
    _highState.imu.quaternion[1] = msg.pose[robot_index].orientation.x;
    _highState.imu.quaternion[2] = msg.pose[robot_index].orientation.y;
    _highState.imu.quaternion[3] = msg.pose[robot_index].orientation.z;

    _highState.imu.gyroscope[0] = msg.twist[robot_index].angular.x;
    _highState.imu.gyroscope[1] = msg.twist[robot_index].angular.y;
    _highState.imu.gyroscope[2] = msg.twist[robot_index].angular.z;
}

void BearIO::LhipCallback(const westwood_legged_msgs::MotorState& msg)
{
    _highState.motorState[0].mode = msg.mode;
    _highState.motorState[0].q = msg.q;
    _highState.motorState[0].dq = msg.dq;
    _highState.motorState[0].tauEst = msg.tauEst;
}

void BearIO::Lhip2Callback(const westwood_legged_msgs::MotorState& msg)
{
    _highState.motorState[1].mode = msg.mode;
    _highState.motorState[1].q = msg.q;
    _highState.motorState[1].dq = msg.dq;
    _highState.motorState[1].tauEst = msg.tauEst;
}

void BearIO::LthighCallback(const westwood_legged_msgs::MotorState& msg)
{
    _highState.motorState[2].mode = msg.mode;
    _highState.motorState[2].q = msg.q;
    _highState.motorState[2].dq = msg.dq;
    _highState.motorState[2].tauEst = msg.tauEst;
}

void BearIO::LcalfCallback(const westwood_legged_msgs::MotorState& msg)
{
    _highState.motorState[3].mode = msg.mode;
    _highState.motorState[3].q = msg.q;
    _highState.motorState[3].dq = msg.dq;
    _highState.motorState[3].tauEst = msg.tauEst;
}

void BearIO::LtoeCallback(const westwood_legged_msgs::MotorState& msg)
{
    _highState.motorState[4].mode = msg.mode;
    _highState.motorState[4].q = msg.q;
    _highState.motorState[4].dq = msg.dq;
    _highState.motorState[4].tauEst = msg.tauEst;
}

void BearIO::RhipCallback(const westwood_legged_msgs::MotorState& msg)
{
    _highState.motorState[5].mode = msg.mode;
    _highState.motorState[5].q = msg.q;
    _highState.motorState[5].dq = msg.dq;
    _highState.motorState[5].tauEst = msg.tauEst;
}

void BearIO::Rhip2Callback(const westwood_legged_msgs::MotorState& msg)
{
    _highState.motorState[6].mode = msg.mode;
    _highState.motorState[6].q = msg.q;
    _highState.motorState[6].dq = msg.dq;
    _highState.motorState[6].tauEst = msg.tauEst;
}

void BearIO::RthighCallback(const westwood_legged_msgs::MotorState& msg)
{
    _highState.motorState[7].mode = msg.mode;
    _highState.motorState[7].q = msg.q;
    _highState.motorState[7].dq = msg.dq;
    _highState.motorState[7].tauEst = msg.tauEst;
}

void BearIO::RcalfCallback(const westwood_legged_msgs::MotorState& msg)
{
    _highState.motorState[8].mode = msg.mode;
    _highState.motorState[8].q = msg.q;
    _highState.motorState[8].dq = msg.dq;
    _highState.motorState[8].tauEst = msg.tauEst;
}

void BearIO::RtoeCallback(const westwood_legged_msgs::MotorState& msg)
{
    _highState.motorState[9].mode = msg.mode;
    _highState.motorState[9].q = msg.q;
    _highState.motorState[9].dq = msg.dq;
    _highState.motorState[9].tauEst = msg.tauEst;
}

void BearIO::ContactCallback(const std_msgs::UInt8 & msg)
{
    for(int i = 0; i < 4; i++) bfr_contact[i] = contact[i];
    uint8_t b = 0x03;
    for(int i = 0; i < 4; i++){
        if( b & msg.data){
            floatTime[i] = 0;
            contact[i] = 1;
        }else{
            floatTime[i]++;
            contact[i] = 0;
        }
        b = b << 2;
    }
}
