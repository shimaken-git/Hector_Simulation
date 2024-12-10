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

/* Authors: Ryan Shim */

#include "bear_actuator_ros/hardware_interface.h"
#include "cbear/bear_sdk.h"
#include "cbear/bear_macro.h"
#include "gim/gim.hpp"
#include <unistd.h>
#include <math.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/LU>

namespace bear_actuator_ros
{
HardwareInterface::HardwareInterface(ros::NodeHandle nh, ros::NodeHandle private_nh)
: node_handle_(nh), priv_node_handle_(private_nh), bear_handle("", 8000000), torque_constant(0.35)
{
  /************************************************************
  ** Initialize ROS parameters
  ************************************************************/
  port_name_ = priv_node_handle_.param<std::string>("usb_port", "/dev/ttyUSB0");
  baud_rate_ = priv_node_handle_.param<int32_t>("baud_rate", 8000000);
  yaml_file_ = priv_node_handle_.param<std::string>("yaml_file", "");
  interface_ = priv_node_handle_.param<std::string>("interface", "position");

  /************************************************************
  ** Register Interfaces
  ************************************************************/
  registerActuatorInterfaces();
  makeActuatorList();
  makeAJmatrix();
  registerControlInterfaces();
}

HardwareInterface::~HardwareInterface()
{
  for(auto act : bearActuator_){
    if(bearValid_[act.first]){
      bear_handle.SetTorqueEnable((uint8_t)act.second, 0);
    }
  }
  uint8_t err;
  for(auto act : gimActuator_){
    if(gimValid_[act.first]){
      gim_handle.Off((uint16_t)act.second, err);
    }
  }

  bear_handle.disconnect();
}

void HardwareInterface::makeAJmatrix()
{
  int col = bearActuator_.size() + gimActuator_.size();
  int row = robotJoint_.size();

  Eigen::MatrixXd a2jMat(row, col);
  a2jMat = Eigen::MatrixXd::Zero(row, col);

  int r = 0;
  for(auto i : robotJoint_){
    std::cout << i.first << std::endl;
    for(auto a : i.second){
      int c;
      if(bearActuator_.count(a.item_name))
        c = idIndex_[bearActuator_[a.item_name]];
      else if(gimActuator_.count(a.item_name))
        c = gimIdIndex_[gimActuator_[a.item_name]];
      std::cout << a.item_name << " " << a.value << " joint_index:" << c << " bear"<< std::endl;
      a2jMat(r,c) = a.value;
    }
    r++;
  }
  std::cout << " a2jMat:" << std::endl << a2jMat << std::endl;

  j2aMat = a2jMat.inverse();
  std::cout << " j2aMat:" << std::endl << j2aMat << std::endl;

}

void HardwareInterface::makeActuatorList()
{
  std::cout << "makeActuatorList()" << std::endl;
  //make idList & idIndex_
  int index = 0;
  for(auto &act : bearActuator_){
    std::cout << act.first << " " << act.second << " " << bearValid_[act.first] << std::endl;
    if(bearValid_[act.first]){
      idList.push_back((uint8_t)act.second);
    }
    idIndex_[act.second] = index++;
  }
  std::cout << "idList.size()=" << idList.size() << std::endl;
  for(auto &id : idList) std::cout << (int)id << " ";
  std::cout << std::endl;
  std::cout << "idIndex_.size()=" << idIndex_.size() << std::endl;
  for(auto &a : idIndex_) std::cout << "ID "<< a.first << " index " << a.second << std::endl;

  //make gimIdList & gimIdIndex_
  // index = 0;
  for(auto &act : gimActuator_){
    std::cout << act.first << " " << act.second << " " << gimValid_[act.first] << std::endl;
    if(gimValid_[act.first]){
      gimIdList.push_back((uint16_t)act.second);
    }
    gimIdIndex_[act.second] = index++;
  }
  std::cout << "gimIdList.size()=" << gimIdList.size() << std::endl;
  for(auto &id : gimIdList) std::cout << (int)id << " ";
  std::cout << std::endl;
  std::cout << "gimIdIndex_.size()=" << gimIdIndex_.size() << std::endl;
  for(auto &a : gimIdIndex_) std::cout << "ID "<< a.first << " index " << a.second << std::endl;
  
    //make convertA2Jset
  for(auto i : robotJoint_){
    std::cout << i.first << std::endl;
    std::map<uint32_t, double> s;
    for(auto j : i.second){
      std::cout << "  " << j.item_name << " " << j.value << std::endl;
      int32_t idx;
      // if(idIndex_.count(bearActuator_[j.item_name])){
      if(bearActuator_.count(j.item_name)){
        idx = idIndex_[bearActuator_[j.item_name]];  //row joint index
      // }else if(gimIdIndex_.count(gimActuator_[j.item_name])){
      }else if(gimActuator_.count(j.item_name)){
        idx = gimIdIndex_[gimActuator_[j.item_name]];
      }
      s[idx] = j.value;
    }
    convertA2Jset.push_back(s);
  }
  std::cout << "convertA2Jset" << std::endl;
  int idx = 0;
  for(auto i : convertA2Jset){
    std::cout << idx++ << std::endl;
    for(auto ite = i.begin(); ite != i.end(); ite++){
      std::cout << "  " << ite->first << " " << ite->second << std::endl;
    }
  }
  if(interface_ == "position"){
    write_add = std::vector<uint8_t>{bear_macro::GOAL_POSITION};
  }else if(interface_ == "velocity"){
    write_add = std::vector<uint8_t>{bear_macro::GOAL_VELOCITY};
  }else if(interface_ == "effort"){
    write_add = std::vector<uint8_t>{bear_macro::GOAL_IQ};
  }
  read_add = std::vector<uint8_t>{bear_macro::PRESENT_POSITION, bear_macro::PRESENT_VELOCITY, bear_macro::PRESENT_IQ};
}

void HardwareInterface::registerActuatorInterfaces()
{  
  // dxl_wb_ = new DynamixelWorkbench;
  bool result = false;

  result = initPort(port_name_, baud_rate_);
  if (result == false){
    ROS_ERROR("Please check USB port name");
    return;
  }

  result = gimConnect();
  if (result == false){
    ROS_ERROR("Please check CAN interface");
    return;
  }

  result = getActuatorInfo(yaml_file_);
  if (result == false){
    ROS_ERROR("Please check YAML file");
    return;
  }

  result = loadActuators();
  if (result == false){
    ROS_ERROR("Please check Actuator ID or BaudRate");
    return;
  }

  result = initActuators();
  if (result == false){
    ROS_ERROR("Please check control table");
    return;
  }

  gimCalibration();
}

bool HardwareInterface::initPort(const std::string port_name, const uint32_t baud_rate)
{
  bool result = false;
  char log[256];

  bear_handle = bear::BEAR(port_name.c_str(), baud_rate);

  result = bear_handle.connect(log);
  // result = dxl_wb_->init(port_name.c_str(), baud_rate, &log);
  if (result == false)
  {
    ROS_ERROR("%s", log);
  }else{
    ROS_INFO("%s", log);
  }

  return result;
}

bool HardwareInterface::gimConnect()
{
  int result;
  char log[256];

  result = gim_handle.connect();
  if (result != 1)
  {
    ROS_ERROR("can connect fault");
    return false;
  }
  ROS_INFO("can connect");
  return true;
}

bool HardwareInterface::getActuatorInfo(const std::string yaml_file)
{
  std::cout << "getActuatorInfo" << std::endl;

  YAML::Node actuator_yaml;
  actuator_yaml = YAML::LoadFile(yaml_file.c_str());

  if (actuator_yaml == NULL)
    return false;

  if(actuator_yaml.size() == 2){
    std::cout << "row_joint" << std::endl;
    YAML::Node block = actuator_yaml["row_joint"];
    for (YAML::const_iterator it_block = block.begin(); it_block != block.end(); it_block++){
      std::string name = it_block->first.as<std::string>();
      if (name.size() == 0)
      {
        continue;
      }

      YAML::Node item = block[name];
      std::cout << "  name " << name << std::endl;
      std::string type_name;
      for (YAML::const_iterator it_item = item.begin(); it_item != item.end(); it_item++){
        std::string item_name = it_item->first.as<std::string>();
        if(item_name == "type"){
          type_name = it_item->second.as<std::string>();
          break;
        }
      }
      std::cout << "    type:" << type_name << std::endl;
      if(type_name == "bear"){
        std::vector<ItemValueD> info_vec;
        for (YAML::const_iterator it_item = item.begin(); it_item != item.end(); it_item++){
          std::string item_name = it_item->first.as<std::string>();
          if (item_name == "ID"){
            int32_t value = it_item->second.as<int32_t>();
            bearActuator_[name] = value;
            bearValid_[name] = true;
            std::cout << "    " << item_name << ":" << value << std::endl;
          }else if(item_name != "type"){
            double value = it_item->second.as<double>();
            ItemValueD item_value = {item_name, value};
            // std::pair<std::string, ItemValueD> info(name, item_value);
            // bearActuator_info_.push_back(info);
            info_vec.push_back(item_value);
            std::cout << "    " << item_name << ":" << value << std::endl;
          }
        }
        bearActuator_info_[name] = info_vec;
      }else if(type_name == "gim"){
        std::vector<ItemValueD> info_vec;
        for (YAML::const_iterator it_item = item.begin(); it_item != item.end(); it_item++)
        {
          std::string item_name = it_item->first.as<std::string>();
          if (item_name == "ID"){
            int32_t value = it_item->second.as<int32_t>();
            gimActuator_[name] = value;
            gimValid_[name] = true;
            std::cout << "    " << item_name << ":" << value << std::endl;
          }else if(item_name != "type"){
            double value = it_item->second.as<double>();
            ItemValueD item_value = {item_name, value};
            info_vec.push_back(item_value);
            // std::pair<std::string, ItemValueD> info(name, item_value);
            // gimActuator_info_.push_back(info);
          }
          gimActuator_info_[name] = info_vec;
        }
      }
    }

    std::cout << "robot_joint" << std::endl;
    YAML::Node block2 = actuator_yaml["robot_joint"];
    for (YAML::const_iterator it_block = block2.begin(); it_block != block2.end(); it_block++){
      std::string name = it_block->first.as<std::string>();
      // std::string name = it_file->first.as<std::string>();
      if (name.size() == 0)
      {
        continue;
      }
      std::cout << name << std::endl;
      YAML::Node item = block2[name];
      YAML::Node j = item["joint"];
      // std::cout << j[0]["name"] << std::endl;
      std::vector<ItemValueD> jnt_vec;
      for(const auto j_item : j){
        std::string joint = j_item["name"].as<std::string>();
        double value = j_item["coeff"].as<double>();
        ItemValueD item;
        item.item_name = joint;
        item.value = value;
        std::cout << "  " << j_item["name"] << " " << item.item_name << " " << item.value << std::endl;
        jnt_vec.push_back(item);
      }
      robotJoint_[name] = jnt_vec;
    }
  }

  return true;
}

bool HardwareInterface::loadActuators(void)
{
  bool result = true;

  for (auto &act:bearActuator_)
  {
    // uint16_t model_number = 0;
    if ((bool)bear_handle.ping((uint8_t)act.second) == false)
    {
      ROS_ERROR("Can't find Actuator ID '%d' error code %d", act.second, bear_handle.GetErrorCode());
      result = false;
      bearValid_[act.first] = false;
    }else{
      // ROS_INFO("Name : %s, ID : %d, Model Number : %d", act.first.c_str(), act.second, model_number);
      ROS_INFO("Name : %s, ID : %d", act.first.c_str(), act.second);
    }
  }

  for (auto &act:gimActuator_)
  {
    // uint16_t model_number = 0;
    uint16_t id = (uint16_t)act.second;
    if ((bool)gim_handle.ping(id) == false)
    {
      ROS_ERROR("Can't find Actuator ID '%d' ", act.second);
      result = false;
      gimValid_[act.first] = false;
    }else{
      ROS_INFO("Name : %s, ID : %d", act.first.c_str(), act.second);
      gim_handle.EntryActuator(id);
      gim_handle.EntryZeropos(id, 0);
    }
  }

  return result;
}

bool HardwareInterface::initActuators(void)
{
  const char* log;

  for(auto const& act:bearActuator_){
    std::cout << "torque off act " << act.first << " " << act.second << std::endl;
    if(bearValid_[act.first]){
      if(!bear_handle.SetTorqueEnable((uint8_t)act.second, 0)){
        ROS_ERROR("BEAR ACTUATOR ERROR %s %d", act.first.c_str(), act.second);
      }
    }
    for(auto a : bearActuator_info_[act.first]){
      std::cout << a.item_name << " " << a.value << std::endl;
      if(a.item_name == "limit_i_max"){
        if(!bear_handle.SetLimitIMax((uint8_t)act.second, a.value)){
          ROS_ERROR("BEAR ACTUATOR ERROR %s %d %s %f", act.first.c_str(), act.second, a.item_name.c_str(), a.value);
        }
      }
    }
  }

  uint8_t err;
  for(auto const& act: gimActuator_){
    std::cout << "gim actuator " << act.first << " " << act.second << std::endl;
    if(gimValid_[act.first]){
      if(gim_handle.Off((uint16_t)act.second, err) < 0){
        ROS_ERROR("GIM ACTUATOR ERROR %s %d", act.first.c_str(), act.second);
      }
    } 
  }


  // Torque On after setting up all servo
  for (auto const& act:bearActuator_){
    if(bearValid_[act.first]){
      std::cout << "torque on bear " << act.first << " " << act.second << std::endl;
      if(!bear_handle.SetTorqueEnable((uint8_t)act.second, 1)){
        ROS_ERROR("BEAR ACTUATOR ERROR %s %d", act.first.c_str(), act.second);
      }
    }
  }
  for(auto const& act: gimActuator_){
    std::cout << "torque on gim " << act.first << " " << act.second << std::endl;
    if(gimValid_[act.first]){
      if(gim_handle.On((uint16_t)act.second, err) < 0){
        ROS_ERROR("GIM ACTUATOR ERROR %s %d", act.first.c_str(), act.second);
      }
    } 
  }
  return true;
}

void HardwareInterface::registerControlInterfaces()
{
  std::cout << "registerControlInterfaces" << std::endl;
  // resize vector
  uint8_t joint_size = robotJoint_.size();
  joints_.resize(joint_size);

  int idx = 0;
  for (auto iter = robotJoint_.begin(); iter != robotJoint_.end(); iter++)
  {
    // initialize joint vector
    Joint joint;
    // int idx = idIndex_[iter->second];
    joints_[idx] = joint;
    ROS_INFO("joint_name : %s", iter->first.c_str());
    // connect and register the joint state interface
    hardware_interface::JointStateHandle joint_state_handle(iter->first.c_str(),
                                                            &joints_[idx].position,
                                                            &joints_[idx].velocity,
                                                            &joints_[idx].effort);
    joint_state_interface_.registerHandle(joint_state_handle);

    // connect and register the joint position, velocity and effort interface
    hardware_interface::JointHandle position_joint_handle(joint_state_handle, &joints_[idx].position_command);
    position_joint_interface_.registerHandle(position_joint_handle);
    hardware_interface::JointHandle velocity_joint_handle(joint_state_handle, &joints_[idx].velocity_command);
    velocity_joint_interface_.registerHandle(velocity_joint_handle);
    hardware_interface::JointHandle effort_joint_handle(joint_state_handle, &joints_[idx].effort_command);
    effort_joint_interface_.registerHandle(effort_joint_handle);
    idx++;
  }

  registerInterface(&joint_state_interface_);
  registerInterface(&position_joint_interface_);
  registerInterface(&velocity_joint_interface_);
  registerInterface(&effort_joint_interface_);

  std::vector<std::string> name_list = getNames();
  for(int j=0; j < name_list.size(); j++){
    ROS_INFO("%s", name_list[j].c_str());
    std::vector<std::string> resource_list = getInterfaceResources(name_list[j]);
    ROS_INFO("resource_list size %ld", resource_list.size());
    for(int i= 0; i < resource_list.size(); i++){
      ROS_INFO("  %s", resource_list[i].c_str());
    }
  }

}

double HardwareInterface::convertActuator2Joint(int idx, double *data)
{
  double j = 0;
  for(auto i : convertA2Jset[idx]){
    j += data[i.first] * i.second;
  }
  return j;
}

void HardwareInterface::gimCalibration()
{
    for(auto a: gimActuator_){
      if(gimValid_[a.first]){
        int16_t id = a.second;
        bool loop = true;
        uint8_t err;
        int32_t result;
        float rpm;
        float lmt;
        std::vector<ItemValueD> param_vec = gimActuator_info_[a.first];
        for(auto i : param_vec){
          if(i.item_name == "cal_rpm"){
            rpm = i.value;
            break;
          }
        }
        for(auto i : param_vec){
          if(i.item_name == "limit"){
            lmt = i.value;
            break;
          }
        }
        gim_handle.On(id, err);
        gim_handle.SetVelocity(id, rpm, 10, err);
        sleep(1.0);
        while(loop){
            if(gim_handle.GetVelocity(id,result) == 0) loop = false;
        }
        float limit = gim_handle.GetPosition(id, result);
        gim_handle.SetVelocity(id, 0, 10, err);
        float zeroPos = limit - lmt;
        printf("id[%d] limit %f zeropos %f \r\n", id, limit, zeroPos);
        gim_handle.Off(id, err);
        gim_handle.SetZeroPosition(id, zeroPos, err);
        gim_handle.EntryZeropos(id, zeroPos);
        sleep(1.0);
        gim_handle.On(id, err);
        gim_handle.SetPosition(id, 0, 500, err);
        sleep(1.0);
        // gim_handle.Off(id, err);
      }
    }
}

void HardwareInterface::read()
{
  static bool first = true;
  bool result = false;
  const char* log = NULL;

  uint8_t act_size = robotJoint_.size();
  double get_position[act_size] = {0};
  double get_velocity[act_size] = {0};
  double get_current[act_size] = {0};

  // uint8_t id_array[act_size];
  // std::string name_array[act_size];
  // uint8_t id_cnt = 0;

  // uint8_t sync_read_handler = 0; // 0 for present position, velocity, current
  // for (auto const& act:bearActuator_)
  // {
  //   name_array[id_cnt] = act.first.c_str();
  //   id_array[id_cnt++] = (uint8_t)act.second;
  // }


  std::vector<std::vector<float> > ret_vec_r;
  ret_vec_r = bear_handle.BulkRead(idList, read_add);
  result = true;
  if(ret_vec_r.size() == 0) result = false;
  uint8_t err = bear_handle.GetErrorCode();
  if (result == false){
    ROS_ERROR("read error  bear_error=%d", err);
  }else{
    // std::cout << std::endl;
    // for(auto v : ret_vec_r) for(auto d : v) std::cout << d << std::endl;
    if(ret_vec_r.size() == idList.size()){
      for(int i = 0; i < ret_vec_r.size(); i++){
        int idx = idIndex_[idList[i]];
        get_position[idx] = ret_vec_r[i][1];
        get_velocity[idx] = ret_vec_r[i][2];
        get_current[idx] = ret_vec_r[i][3] * torque_constant;
        uint8_t err = ret_vec_r[i][4];  // error code
      }
    }else{
      ROS_ERROR("read data fault");
    }
  }
  for(auto id : gimIdList){
    int32_t result;
    int idx = gimIdIndex_[id];
    get_position[idx] = gim_handle.GetPosition(id, result);
    get_velocity[idx] = gim_handle.GetVelocity(id, result);
    get_current[idx] = gim_handle.GetTorque(id, result);
  }
  for(uint8_t idx = 0; idx < act_size; idx++){
    // Position
    joints_[idx].position = convertActuator2Joint(idx, get_position);
    // Velocity
    joints_[idx].velocity = convertActuator2Joint(idx, get_velocity);
    // Effort
    joints_[idx].effort = convertActuator2Joint(idx, get_current);
    ROS_INFO("[%d] pos %f vel %f cur %f", idx, joints_[idx].position, joints_[idx].velocity, joints_[idx].effort);
  }
  if(first){
    for(uint8_t idx = 0; idx < act_size; idx++){
      joints_[idx].position_command = joints_[idx].position;
    }
    first = false;
  }
}

void HardwareInterface::write()
{
  bool result = false;

  int col = bearActuator_.size() + gimActuator_.size();
  int row = robotJoint_.size();

  Eigen::MatrixXd joint_data(col, 1);
  for(int i = 0; i < col; i++) joint_data(i, 0) = joints_[i].position_command;
  Eigen::MatrixXd actuator_data(col, 1);
  actuator_data = j2aMat * joint_data;
  // std::cout << "actuator_data " << actuator_data << std::endl;
  if (strcmp(interface_.c_str(), "position") == 0){
    std::vector<std::vector<float>> data;
    for(auto id : idList){
      int idx = idIndex_[id];
      std::vector<float> _data;
      _data.push_back(actuator_data(idx, 0));
      data.push_back(_data);
    }
    if(!bear_handle.BulkWrite(idList, write_add, data)){
      ROS_ERROR("BEAR ACTUATOR WRITE ERROR");
    }
  }
  else if (strcmp(interface_.c_str(), "effort") == 0){
    std::vector<std::vector<float>> data;
    for(auto id : idList){
      int idx = idIndex_[id];
      std::vector<float> _data;
      _data.push_back(joints_[idx].effort_command);
      data.push_back(_data);
    }
    if(!bear_handle.BulkWrite(idList, write_add, data)){
      ROS_ERROR("BEAR ACTUATOR WRITE ERROR");
    }
  }

  uint8_t err;
  if (strcmp(interface_.c_str(), "position") == 0){
    for(auto id : gimIdList){
      int idx = gimIdIndex_[id];
      if(gim_handle.SetPosition((uint16_t)id, actuator_data(idx, 0), 10, err) < 0){
        ROS_ERROR("GIM ACTUATOR WRITE ERROR %d err:%d", id, err);
      }
    }
  }
  else if (strcmp(interface_.c_str(), "effort") == 0){
    for(auto id : gimIdList){
      int idx = gimIdIndex_[id];
      if(gim_handle.SetPosition((uint16_t)id, joints_[idx].effort_command, 10, err) < 0){
        ROS_ERROR("GIM ACTUATOR WRITE ERROR %d err:%d", id, err);
      }
    }
  }
}

void HardwareInterface::torque(bool torque)
{
  uint8_t err;
  if(torque){
    for (auto const& id : idList){
      if(!bear_handle.SetTorqueEnable(id, 1)){
        ROS_ERROR("BEAR ACTUATOR ERROR %d", id);
      }
    }
    for(auto const& id : gimIdList){
      if(gim_handle.On(id, err) < 0){
        ROS_ERROR("GIM ACTUATOR ERROR %d", id);
      }
    }
  }else{
    for (auto const& id : idList){
      if(!bear_handle.SetTorqueEnable(id, 0)){
        ROS_ERROR("BEAR ACTUATOR ERROR %d", id);
      }
    }
    for(auto const& id : gimIdList){
      if(gim_handle.Off(id, err) < 0){
        ROS_ERROR("GIM ACTUATOR ERROR %d", id);
      }
    }
  }
}

}
