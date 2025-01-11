/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  Copyright (c) 2012, hiDOF, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/*
 Author: Vijay Pradeep
 Contributors: Jonathan Bohren, Wim Meeussen, Dave Coleman
 Desc: Effort(force)-based position controller using basic PID loop
*/

#include <bear_effort_controllers/joint_position_controller.h>
#include <angles/angles.h>
#include <pluginlib/class_list_macros.hpp>

namespace bear_effort_controllers {

JointPositionController::JointPositionController()
  : loop_count_(0)
{}

JointPositionController::~JointPositionController()
{
  sub_command_.shutdown();
}

bool JointPositionController::init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n)
{
  // Get joint name from parameter server
  std::string joint_name;
  if (!n.getParam("joint", joint_name))
  {
    ROS_ERROR("No joint given (namespace: %s)", n.getNamespace().c_str());
    return false;
  }
  ROS_INFO("joint name : %s", joint_name.c_str());
  // Load PID Controller using gains set on parameter server
  if (!pid_controller_.init(ros::NodeHandle(n, "pid")))
    return false;

  // Start realtime state publisher
  // controller_state_publisher_.reset(
  //   new realtime_tools::RealtimePublisher<control_msgs::JointControllerState>(n, "state", 1));
  controller_state_publisher_.reset(new realtime_tools::RealtimePublisher<westwood_legged_msgs::MotorState>(n, "state", 1));
  ROS_INFO("state set");
  // Start command subscriber
  // sub_command_ = n.subscribe<std_msgs::Float64>("command", 1, &JointPositionController::setCommandCB, this);
  sub_command_ = n.subscribe("command", 1, &JointPositionController::setCommandCB, this);
  ROS_INFO("command set");

  // Get joint handle from hardware interface
  joint_ = robot->getHandle(joint_name);
  ROS_INFO("handle get");

  // Get URDF info about joint
  urdf::Model urdf;
  if (!urdf.initParamWithNodeHandle("robot_description", n))
  {
    ROS_ERROR("Failed to parse urdf file");
    return false;
  }
  joint_urdf_ = urdf.getJoint(joint_name);
  ROS_INFO("joint parameter get");
  if (!joint_urdf_)
  {
    ROS_ERROR("Could not find joint '%s' in urdf", joint_name.c_str());
    return false;
  }

  return true;
}

void JointPositionController::setGains(const double &p, const double &i, const double &d, const double &i_max, const double &i_min, const bool &antiwindup)
{
  pid_controller_.setGains(p,i,d,i_max,i_min,antiwindup);
}

void JointPositionController::getGains(double &p, double &i, double &d, double &i_max, double &i_min, bool &antiwindup)
{
  pid_controller_.getGains(p,i,d,i_max,i_min,antiwindup);
}

void JointPositionController::getGains(double &p, double &i, double &d, double &i_max, double &i_min)
{
  bool dummy;
  pid_controller_.getGains(p,i,d,i_max,i_min,dummy);
}

void JointPositionController::printDebug()
{
  pid_controller_.printValues();
}

std::string JointPositionController::getJointName()
{
  return joint_.getName();
}

double JointPositionController::getPosition()
{
  return joint_.getPosition();
}

// Set the joint position command
void JointPositionController::setCommand(double pos_command)
{
  ROS_ERROR("setCommand(double) Dont Call function");
  // command_struct_.position_ = pos_command;
  // command_struct_.has_velocity_ = false; // Flag to ignore the velocity command since our setCommand method did not include it

  // // the writeFromNonRT can be used in RT, if you have the guarantee that
  // //  * no non-rt thread is calling the same function (we're not subscribing to ros callbacks)
  // //  * there is only one single rt thread
  // command_.writeFromNonRT(command_struct_);
}

// Set the joint position command with a velocity command as well
void JointPositionController::setCommand(double pos_command, double vel_command)
{
  ROS_ERROR("setCommand(double, double) Dont Call function");
  // command_struct_.position_ = pos_command;
  // command_struct_.velocity_ = vel_command;
  // command_struct_.has_velocity_ = true;

  // command_.writeFromNonRT(command_struct_);
}

void JointPositionController::starting(const ros::Time& time)
{
  double init_pos = joint_.getPosition();               //pos_commandを現在positionで初期化

  // Make sure joint is within limits if applicable
  enforceJointLimits(init_pos);

  // command_struct_.position_ = pos_command;
  // command_struct_.has_velocity_ = false;
  // command_.initRT(command_struct_);
  lastCmd.q = init_pos;
  lastState.q = init_pos;
  lastCmd.dq = 0;
  lastState.dq = 0;
  lastCmd.tau = 0;
  lastState.tauEst = 0;
  lastCmd.mode = PMSM;
  command.initRT(lastCmd);

  pid_controller_.reset();
}

void JointPositionController::update(const ros::Time& time, const ros::Duration& period)
{
  // command_struct_ = *(command_.readFromRT());
  // double command_position = command_struct_.position_;
  // double command_velocity = command_struct_.velocity_;
  // bool has_velocity_ =  command_struct_.has_velocity_;
  // double error, vel_error;
  lastCmd = *(command.readFromRT());

  // double current_position = joint_.getPosition();     //getPosision()で得られるのはドライバーread()で更新した現在データ

  // ROS_INFO("joint name %s  current_pos: %f vel: %f has_velocity_ %d", joint_.getName().c_str(), joint_.getPosition(), joint_.getVelocity(), has_velocity_);
  // Make sure joint is within limits if applicable
  // enforceJointLimits(command_position);

  // // Compute position error
  // if (joint_urdf_->type == urdf::Joint::REVOLUTE){   //limit付き回転型
  //   angles::shortest_angular_distance_with_large_limits(
  //     current_position,
  //     command_position,
  //     joint_urdf_->limits->lower,
  //     joint_urdf_->limits->upper,
  //     error);
  // }else if (joint_urdf_->type == urdf::Joint::CONTINUOUS){  //無限回転
  //   ROS_INFO("joint type CONTINUOUS not implemented.");
  //   // error = angles::shortest_angular_distance(current_position, command_position);
  // }else{ //prismatic   直動型
  //   ROS_INFO("joint type PRISMATIC not implemented.");
  //   // error = command_position - current_position;     //目標positionと現在positionの差
  // }

  // joint_.setCommand(commanded_effort);
  // const double *ptr = joint_.getCommandPtr();
  // double *cmd_ptr = (double*)ptr;
  double *cmd_ptr = (double *)joint_.getCommandPtr();
  cmd_ptr[0] = lastCmd.q;     //position
  cmd_ptr[1] = lastCmd.dq;   //velocity
  cmd_ptr[2] = lastCmd.tau;   //effort
  cmd_ptr[3] = lastCmd.Kp;   //Kp
  cmd_ptr[4] = lastCmd.Kd;   //Kd
  cmd_ptr[5] = 0.0;   //reserve

  // ROS_INFO("cmd data %f %f %f", ptr[0], ptr[1], ptr[2]);

  lastState.q = joint_.getPosition();
  lastState.dq = joint_.getVelocity();
  lastState.tauEst = joint_.getEffort();

  // publish state
  if (loop_count_ % 10 == 0){
    if(controller_state_publisher_ && controller_state_publisher_->trylock()){
      controller_state_publisher_->msg_.q = lastState.q;
      controller_state_publisher_->msg_.dq = lastState.dq;
      controller_state_publisher_->msg_.tauEst = lastState.tauEst;
      controller_state_publisher_->unlockAndPublish();
    }
  }
  loop_count_++;
}

void JointPositionController::setCommandCB(const westwood_legged_msgs::MotorCmdConstPtr& msg)
{
  // setCommand(msg->data);
  lastCmd.mode = msg->mode;
  lastCmd.q = msg->q;
  lastCmd.Kp = msg->Kp;
  lastCmd.dq = msg->dq;
  lastCmd.Kd = msg->Kd;
  lastCmd.tau = msg->tau;
  // the writeFromNonRT can be used in RT, if you have the guarantee that
  //  * no non-rt thread is calling the same function (we're not subscribing to ros callbacks)
  //  * there is only one single rt thread
  command.writeFromNonRT(lastCmd);
}

// Note: we may want to remove this function once issue https://github.com/ros/angles/issues/2 is resolved
void JointPositionController::enforceJointLimits(double &command)
{
  // Check that this joint has applicable limits
  if (joint_urdf_->type == urdf::Joint::REVOLUTE || joint_urdf_->type == urdf::Joint::PRISMATIC)
  {
    if( command > joint_urdf_->limits->upper ) // above upper limnit
    {
      command = joint_urdf_->limits->upper;
    }
    else if( command < joint_urdf_->limits->lower ) // below lower limit
    {
      command = joint_urdf_->limits->lower;
    }
  }
}

} // namespace

PLUGINLIB_EXPORT_CLASS( bear_effort_controllers::JointPositionController, controller_interface::ControllerBase)
