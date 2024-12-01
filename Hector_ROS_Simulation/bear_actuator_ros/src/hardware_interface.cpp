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
#include <unistd.h>
#include <math.h>

namespace bear_actuator_ros
{
HardwareInterface::HardwareInterface(ros::NodeHandle nh, ros::NodeHandle private_nh)
: node_handle_(nh), priv_node_handle_(private_nh), bear_handle("", 8000000)
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
  registerControlInterfaces();
}

void HardwareInterface::makeActuatorList()
{
  std::cout << "makeActuatorList()" << std::endl;
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
  
  for(auto i : robotJoint_){
    std::cout << i.first << std::endl;
    std::map<uint32_t, double> s;
    for(auto j : i.second){
      std::cout << "  " << j.item_name << " " << j.value << std::endl;
      int32_t idx = idIndex_[bearActuator_[j.item_name]];  //row joint index
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
  write_add = std::vector<uint8_t>{bear_macro::GOAL_VELOCITY, bear_macro::GOAL_POSITION};
  read_add = std::vector<uint8_t>{bear_macro::PRESENT_POSITION, bear_macro::PRESENT_VELOCITY, bear_macro::PRESENT_IQ};
}

void HardwareInterface::registerActuatorInterfaces()
{  
  // dxl_wb_ = new DynamixelWorkbench;
  bool result = false;

  result = initPort(port_name_, baud_rate_);
  if (result == false)
  {
    ROS_ERROR("Please check USB port name");
    return;
  }

  result = getActuatorInfo(yaml_file_);
  if (result == false)
  {
    ROS_ERROR("Please check YAML file");
    return;
  }

  result = loadActuators();
  if (result == false)
  {
    ROS_ERROR("Please check Actuator ID or BaudRate");
    return;
  }

  result = initActuators();
  if (result == false)
  {
    ROS_ERROR("Please check control table");
    return;
  }

  result = initControlItems();
  if (result == false)
  {
    ROS_ERROR("Please check control items");
    return;
  }

  result = initSDKHandlers();
  if (result == false)
  {
    ROS_ERROR("Failed to set Dynamixel SDK Handler");
    return;
  }


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

bool HardwareInterface::getActuatorInfo(const std::string yaml_file)
{
  std::cout << "getActuatorInfo" << std::endl;

  YAML::Node actuator_yaml;
  actuator_yaml = YAML::LoadFile(yaml_file.c_str());

  if (actuator_yaml == NULL)
    return false;

  // for (YAML::const_iterator it_file = actuator_yaml.begin(); it_file != actuator_yaml.end(); it_file++){
  std::cout << actuator_yaml.size() << std::endl;
  if(actuator_yaml.size() == 2){
    YAML::Node block = actuator_yaml["row_joint"];
    for (YAML::const_iterator it_block = block.begin(); it_block != block.end(); it_block++){
      std::string name = it_block->first.as<std::string>();
      // std::string name = it_file->first.as<std::string>();
      if (name.size() == 0)
      {
        continue;
      }

      YAML::Node item = block[name];
      for (YAML::const_iterator it_item = item.begin(); it_item != item.end(); it_item++)
      {
        std::string item_name = it_item->first.as<std::string>();
        int32_t value = it_item->second.as<int32_t>();

        if (item_name == "ID"){
          bearActuator_[name] = value;
          bearValid_[name] = true;
        }

        ItemValue item_value = {item_name, value};
        std::pair<std::string, ItemValue> info(name, item_value);

        bearActuator_info_.push_back(info);
      }
    }

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

  return result;
}

bool HardwareInterface::initActuators(void)
{
  const char* log;

  for (auto const& act:bearActuator_)
  {
    std::cout << "torque off act " << act.first << " " << act.second << std::endl;
    // dxl_wb_->torqueOff((uint8_t)act.second);

    for (auto const& info:bearActuator_info_)
    {
      if (act.first == info.first)
      {
        if (info.second.item_name != "ID" && info.second.item_name != "Baud_Rate")
        {
          std::cout << "info.second.item_name " << info.second.item_name << std::endl;
          // bool result = dxl_wb_->itemWrite((uint8_t)act.second, info.second.item_name.c_str(), info.second.value, &log);
          // if (result == false)
          // {
          //   ROS_ERROR("%s", log);
          //   ROS_ERROR("Failed to write value[%d] on items[%s] to Dynamixel[Name : %s, ID : %d]", info.second.value, info.second.item_name.c_str(), act.first.c_str(), act.second);
          //   return false;
          // }
        }
      }
    }
  }

  // Torque On after setting up all servo
  for (auto const& act:bearActuator_)
    std::cout << "torque on act " << act.first << " " << act.second << std::endl;
    // dxl_wb_->torqueOn((uint8_t)act.second);

  return true;
}

bool HardwareInterface::initControlItems(void)
{
  bool result = false;
  const char* log = NULL;

  auto it = bearActuator_.begin();

  //サーボのレジスタの登録かな？
  
  // const ControlItem *goal_position = dxl_wb_->getItemInfo(it->second, "Goal_Position");
  // if (goal_position == NULL) return false;

  // const ControlItem *goal_velocity = dxl_wb_->getItemInfo(it->second, "Goal_Velocity");
  // if (goal_velocity == NULL)  goal_velocity = dxl_wb_->getItemInfo(it->second, "Moving_Speed");
  // if (goal_velocity == NULL)  return false;

  // const ControlItem *goal_current = dxl_wb_->getItemInfo(it->second, "Goal_Current");
  // if (goal_current == NULL) goal_current = dxl_wb_->getItemInfo(it->second, "Present_Load");
  // if (goal_current == NULL) return false;

  // const ControlItem *present_position = dxl_wb_->getItemInfo(it->second, "Present_Position");
  // if (present_position == NULL) return false;

  // const ControlItem *present_velocity = dxl_wb_->getItemInfo(it->second, "Present_Velocity");
  // if (present_velocity == NULL)  present_velocity = dxl_wb_->getItemInfo(it->second, "Present_Speed");
  // if (present_velocity == NULL) return false;

  // const ControlItem *present_current = dxl_wb_->getItemInfo(it->second, "Present_Current");
  // if (present_current == NULL)  present_current = dxl_wb_->getItemInfo(it->second, "Present_Load");
  // if (present_current == NULL) return false;

  // control_items_["Goal_Position"] = goal_position;
  // control_items_["Goal_Velocity"] = goal_velocity;
  // control_items_["Goal_Current"] = goal_current;

  // control_items_["Present_Position"] = present_position;
  // control_items_["Present_Velocity"] = present_velocity;
  // control_items_["Present_Current"] = present_current;

  return true;
}

bool HardwareInterface::initSDKHandlers(void)
{
  bool result = false;
  const char* log = NULL;

  auto it = bearActuator_.begin();

  // result = dxl_wb_->addSyncWriteHandler(control_items_["Goal_Position"]->address, control_items_["Goal_Position"]->data_length, &log);
  // if (result == false)
  // {
  //   ROS_ERROR("%s", log);
  //   return result;
  // }
  // else
  // {
  //   ROS_INFO("%s", log);
  // }

  // result = dxl_wb_->addSyncWriteHandler(control_items_["Goal_Velocity"]->address, control_items_["Goal_Velocity"]->data_length, &log);
  // if (result == false)
  // {
  //   ROS_ERROR("%s", log);
  //   return result;
  // }
  // else
  // {
  //   ROS_INFO("%s", log);
  // }

  // result = dxl_wb_->addSyncWriteHandler(control_items_["Goal_Current"]->address, control_items_["Goal_Current"]->data_length, &log);
  // if (result == false)
  // {
  //   ROS_ERROR("%s", log);
  //   return result;
  // }
  // else
  // {
  //   ROS_INFO("%s", log);
  // }

  // if (dxl_wb_->getProtocolVersion() == 2.0f)
  // {
  //   uint16_t start_address = std::min(control_items_["Present_Position"]->address, control_items_["Present_Current"]->address);

  //   /* As some models have an empty space between Present_Velocity and Present Current, read_length is modified as below.*/
  //   // uint16_t read_length = control_items_["Present_Position"]->data_length + control_items_["Present_Velocity"]->data_length + control_items_["Present_Current"]->data_length;
  //   uint16_t read_length = control_items_["Present_Position"]->data_length + control_items_["Present_Velocity"]->data_length + control_items_["Present_Current"]->data_length+2;

  //   result = dxl_wb_->addSyncReadHandler(start_address,
  //                                         read_length,
  //                                         &log);
  //   if (result == false)
  //   {
  //     ROS_ERROR("%s", log);
  //     return result;
  //   }
  // }

  return result;
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
  // switch(idx){
  //   case 0:
  //     return(data[0]);
  //   case 1:
  //     return((data[2] - data[1])/2);
  //   case 2:
  //     return((data[1] + data[2])/2);
  //   case 3:
  //     return(data[3] - (data[1] + data[2])/2);
  //   case 4:
  //     return(data[4]);
  //   case 5:
  //     return(data[5]);
  //   case 6:
  //     return((data[7] - data[6])/2);
  //   case 7:
  //     return(-(data[6] + data[7])/2);
  //   case 8:
  //     return((data[6] + data[7])/2 - data[8]);
  //   case 9:
  //     return(-data[9]);
  // }
}

void HardwareInterface::read()
{
  bool result = false;
  const char* log = NULL;

  uint8_t act_size = bearActuator_.size();
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
    std::cout << std::endl;
    // for(auto v : ret_vec_r) for(auto d : v) std::cout << d << std::endl;
    if(ret_vec_r.size() == idList.size()){
      for(int i = 0; i < ret_vec_r.size(); i++){
        int idx = idIndex_[idList[i]];
        get_position[idx] = ret_vec_r[i][1];
        get_velocity[idx] = ret_vec_r[i][2];
        get_current[idx] = ret_vec_r[i][3];
        uint8_t err = ret_vec_r[i][4];  // error code
      }
    }else{
      ROS_ERROR("read data fault");
    }
  }
  for(uint8_t idx = 0; idx < act_size; idx++){
    // Position
    joints_[idx].position = convertActuator2Joint(idx, get_position);
    // Velocity
    joints_[idx].velocity = convertActuator2Joint(idx, get_velocity);
    // Effort
    joints_[idx].effort = convertActuator2Joint(idx, get_current);
    joints_[idx].position_command = joints_[idx].position;
    ROS_INFO("[%d] pos %f vel %f cur %f", idx, joints_[idx].position, joints_[idx].velocity, joints_[idx].effort);
  }
}

void HardwareInterface::write()
{
  bool result = false;
  // const char* log = NULL;
  const char* log = "none";

  uint8_t id_array[bearActuator_.size()];
  uint8_t id_cnt = 0;

  int32_t bearActuator_position[bearActuator_.size()];
  int32_t bearActuator_velocity[bearActuator_.size()];
  int32_t bearActuator_effort[bearActuator_.size()];

  if (strcmp(interface_.c_str(), "position") == 0)
  {
    for (auto const& act:bearActuator_)
    {
      id_array[id_cnt] = (uint8_t)act.second;
      // dynamixel_position[id_cnt] = dxl_wb_->convertRadian2Value((uint8_t)act.second, joints_[(uint8_t)act.second-1].position_command);

      if (strcmp(act.first.c_str(), "gripper") == 0)
        // dynamixel_position[id_cnt] = dxl_wb_->convertRadian2Value((uint8_t)act.second, joints_[(uint8_t)act.second-1].position_command * 150.0);
      id_cnt ++;
    }
    uint8_t sync_write_handler = 0; // 0: position, 1: velocity, 2: effort
    // result = dxl_wb_->syncWrite(sync_write_handler, id_array, id_cnt, dynamixel_position, 1, &log);
  }
  else if (strcmp(interface_.c_str(), "effort") == 0)
  {
    for (auto const& act:bearActuator_)
    {
      id_array[id_cnt] = (uint8_t)act.second;
      // dynamixel_effort[id_cnt] = dxl_wb_->convertCurrent2Value((uint8_t)act.second, joints_[(uint8_t)act.second-1].effort_command / (1.78e-03));

      if (strcmp(act.first.c_str(), "gripper") == 0)
        // dynamixel_position[id_cnt] = dxl_wb_->convertRadian2Value((uint8_t)act.second, joints_[(uint8_t)act.second-1].position_command * 150.0);
      id_cnt ++;
    }
    uint8_t sync_write_handler = 2; // 0: position, 1: velocity, 2: effort
    // result = dxl_wb_->syncWrite(sync_write_handler, id_array, id_cnt, dynamixel_effort, 1, &log);
  }

  if (result == false)
  {
    ROS_ERROR("%s", log);
  }
}

void HardwareInterface::torque(bool torque)
{
  for (auto const& act:bearActuator_)
  {
    if(torque){
      // dxl_wb_->torqueOn((uint8_t)act.second);
    }else{
      // dxl_wb_->torqueOff((uint8_t)act.second);
    }
  }
}

} // namespace open_manipulator_hw
