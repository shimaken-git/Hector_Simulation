/*******************************************************************************
* Copyright 2020 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

#ifndef BEAR_ACTUATOR_HARDWARE_INTERFACE_H
#define BEAR_ACTUATOR_HARDWARE_INTERFACE_H

#include <yaml-cpp/yaml.h>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/LU>


#include "cbear/bear_sdk.h"
#include "cbear/bear_macro.h"
#include "gim/gim.hpp"

typedef struct _ItemValue
{
  std::string item_name;
  int32_t value;
} ItemValue;

typedef struct _ItemValueD
{
  std::string item_name;
  double value;
} ItemValueD;

typedef struct _WayPoint
{
  double position;
  double velocity;
  double acceleration;
} WayPoint;

typedef struct _Joint
{
  double position;
  double velocity;
  double current;
  double effort;
  double position_command;
  double velocity_command;
  double effort_command;
} Joint;

namespace bear_actuator_ros
{
class HardwareInterface : public hardware_interface::RobotHW
{
 public:
  HardwareInterface(ros::NodeHandle nh, ros::NodeHandle private_nh);
  ~HardwareInterface();

  void read();
  void write();
  void torque(bool);

 private:
  void registerActuatorInterfaces();
  void registerControlInterfaces();
  bool initPort(const std::string port_name, const uint32_t baud_rate);
  bool gimConnect();
  bool getActuatorInfo(const std::string yaml_file);
  bool loadActuators(void);
  bool initActuators(void);
  void makeActuatorList(void);
  void makeAJmatrix(void);
  double convertActuator2Joint(int idx, double *data);
  void gimCalibration();

  // ROS NodeHandle
  ros::NodeHandle node_handle_;
  ros::NodeHandle priv_node_handle_;

  // ROS Parameters
  std::string port_name_;
  int32_t baud_rate_;
  std::string yaml_file_;
  std::string interface_;

  // bear actuator parameters
  double torque_constant;

  // Variables
  
  bear::BEAR bear_handle;
  std::map<std::string, uint32_t> bearActuator_;    //row joint information
  std::map<std::string, bool> bearValid_;           //row joint valid information
  // std::vector<std::pair<std::string, ItemValueD>> bearActuator_info_;
  std::map<std::string, std::vector<ItemValueD>> bearActuator_info_;
  std::map<uint32_t, uint32_t> idIndex_;  // first:ID  second:Index

  gim::GIM gim_handle;
  std::map<std::string, uint32_t> gimActuator_;    //row joint information
  std::map<std::string, bool> gimValid_;           //row joint valid information
  // std::vector<std::pair<std::string, ItemValueD>> gimActuator_info_;
  std::map<std::string, std::vector<ItemValueD>> gimActuator_info_;
  std::map<uint16_t, uint32_t> gimIdIndex_;  // first:ID  second:Index
  std::vector<std::map<uint32_t, double>> convertA2Jset;  //rowJoint -> robotJoint coefficent set
  // std::map<std::string, const ControlItem*> control_items_;
  std::map<std::string, std::vector<ItemValueD>> robotJoint_;    //robot joint information
  std::vector<Joint> joints_;
  std::vector<uint8_t> idList;
  std::vector<uint16_t> gimIdList;
  std::vector<uint8_t> write_add;
  std::vector<uint8_t> read_add;

  Eigen::MatrixXd j2aMat;

  // ROS Control interfaces
  hardware_interface::JointStateInterface joint_state_interface_;
  hardware_interface::PositionJointInterface position_joint_interface_;
  hardware_interface::VelocityJointInterface velocity_joint_interface_;
  hardware_interface::EffortJointInterface effort_joint_interface_;
};

}
#endif
