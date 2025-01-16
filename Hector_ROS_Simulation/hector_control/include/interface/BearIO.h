#ifndef BEARIO_H
#define BEARIO_H

#include "ros/ros.h"
#include <ros/time.h>
#include<boost/array.hpp>
#include "IOInterface.h"
#include <westwood_legged_msgs/LowCmd.h>
#include <westwood_legged_msgs/MotorCmd.h>
#include <westwood_legged_msgs/MotorState.h>
#include <westwood_legged_msgs/HighState.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/ContactsState.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>
#include <string>
#include <std_msgs/UInt8.h>

class BearIO : public IOInterface
{
    public:
        BearIO(std::string robot_name, double _height);
        ~BearIO();
        void sendRecv(const LowlevelCmd *cmd, LowlevelState *state);
    private:
        void sendCmd(const LowlevelCmd *cmd);
        void recvState(LowlevelState *state);
        ros::NodeHandle _nm;
        ros::Subscriber _servo_sub[10], _state_sub, _contact_sub;
        ros::Publisher _servo_pub[10];
        // ros::Publisher _contact_pub[4];
        westwood_legged_msgs::LowCmd _lowCmd;
        westwood_legged_msgs::HighState _highState;
        ros::AsyncSpinner _subSpinner;

        double height;


        std::string _robot_name;
        void initRecv(); // initialize subscribers
        void initSend(); // initialize publishers
    
        void StateCallback(const gazebo_msgs::ModelStates & msg);

        // void ContactLFCallback(const gazebo_msgs::ContactsState & msg);
        // void ContactLBCallback(const gazebo_msgs::ContactsState & msg);
        // void ContactRFCallback(const gazebo_msgs::ContactsState & msg);
        // void ContactRBCallback(const gazebo_msgs::ContactsState & msg);
        void ContactCallback(const std_msgs::UInt8 & msg);

        void LhipCallback(const westwood_legged_msgs::MotorState& msg);
        void Lhip2Callback(const westwood_legged_msgs::MotorState& msg);
        void LthighCallback(const westwood_legged_msgs::MotorState& msg);
        void LcalfCallback(const westwood_legged_msgs::MotorState& msg);
        void LtoeCallback(const westwood_legged_msgs::MotorState& msg);
        void RhipCallback(const westwood_legged_msgs::MotorState& msg);
        void Rhip2Callback(const westwood_legged_msgs::MotorState& msg);
        void RthighCallback(const westwood_legged_msgs::MotorState& msg);
        void RcalfCallback(const westwood_legged_msgs::MotorState& msg);
        void RtoeCallback(const westwood_legged_msgs::MotorState& msg);

};   

#endif