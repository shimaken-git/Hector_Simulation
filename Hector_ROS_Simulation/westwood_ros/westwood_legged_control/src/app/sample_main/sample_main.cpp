//
// Created by msahn on 3/11/21.
//
#ifdef ENABLE_DOCTEST_IN_LIBRARY
#define DOCTEST_CONFIG_IMPLEMENT
// #include "doctest.h"
#endif

#include <iostream>
#include <chrono>

#include "cbear/bear_sdk.h"
#include "cbear/bear_macro.h"

#include <unistd.h>

#include <math.h>

// Settings
#define DEVICENAME "/dev/ttyUSB0"

int main(int argc, char *argv[]) {
  std::cout << "RoMeLa CBEAR sample code." << std::endl;

  char *dev_name = (char *) DEVICENAME;

  // Initialize BEAR instance
  bear::BEAR bear_handle = bear::BEAR(dev_name, 8000000);

  // Sample command
  float ret_val{0.0};
  ret_val = bear_handle.GetPresentPosition(10);
  // bear_handle.SetMode(5,3);
  // bear_handle.SetMode(10,3);
  bear_handle.SetMode(5,2);
  bear_handle.SetMode(10,2);
  std::cout << "mode id 5 " << bear_handle.GetMode(5) << std::endl;
  std::cout << "mode id 10 " << bear_handle.GetMode(10) << std::endl;
  std::cout << "ret_val " << ret_val << std::endl;
  std::cout << "torque enable " << bear_handle.SetTorqueEnable(5, 1) << std::endl;
  std::cout << "torque enable " << bear_handle.SetTorqueEnable(10, 1) << std::endl;

  // Sample bulk read write
  std::vector<uint8_t> mIDs{5, 10};
  // std::vector<uint8_t> write_add{bear_macro::GOAL_VELOCITY, bear_macro::GOAL_POSITION, bear_macro::GOAL_IQ};
  std::vector<uint8_t> write_add{bear_macro::GOAL_VELOCITY, bear_macro::GOAL_POSITION};
  std::vector<uint8_t> read_add{bear_macro::PRESENT_POSITION, bear_macro::PRESENT_VELOCITY, bear_macro::PRESENT_IQ};
  // std::vector<std::vector<float>> data{{0.0, 0.0, 0.0},
  //                                      {0.0, 0.0, 0.0}};
  std::vector<std::vector<float>> data{{0.0, 0.0},
                                       {0.0, 0.0}};
  std::vector<std::vector<float>> ret_vec_rw;
  ret_vec_rw = bear_handle.BulkReadWrite(mIDs, read_add, write_add, data);

  for(int i = 0; i < ret_vec_rw.size(); i++){
    for(float d : ret_vec_rw[i]){
      std::cout << d << std::endl;
    }
    std::cout << ret_vec_rw[i][4] << std::endl;
  }

  sleep(1);

  for(int i = 0; i < 10; i++){
    float pos = 1.7 * (i%2 == 0? 1 : -1);
    std::cout << "pos : " << pos << std::endl;
    // float trq = 0.0 * (i%2 == 0? 1 : -1);
    // data[0] = {0.0, pos, trq};
    // data[1] = {0.0, pos, trq};
    data[0] = {0.0, pos};
    data[1] = {0.0, pos};
    ret_vec_rw = bear_handle.BulkReadWrite(mIDs, read_add, write_add, data);
    usleep(500000);
  }

  sleep(1);
  std::cout << "torque disable " << bear_handle.SetTorqueEnable(5, 0) << std::endl;
  std::cout << "torque disable " << bear_handle.SetTorqueEnable(10, 0) << std::endl;

}
