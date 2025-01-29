#include "../../include/common/PositionVelocityEstimator.h"
#include "../../include/common/robot_select.h"

void CheaterPositionVelocityEstimator::run() {
 // std::cout << "run StateEstimator" << std::endl;
#ifdef FOOTSENSOR
  this->_stateEstimatorData.result->position = this->_stateEstimatorData.result->p_world;
  this->_stateEstimatorData.result->vWorld = this->_stateEstimatorData.result->v_world;
#else
  for(int i = 0; i < 3; i++){
    this->_stateEstimatorData.result->position[i] = this->_stateEstimatorData.lowState->position[i];
    this->_stateEstimatorData.result->vWorld[i] = this->_stateEstimatorData.lowState->vWorld[i];
  }
#endif

  this->_stateEstimatorData.result->vBody=
  this->_stateEstimatorData.result->rBody * this->_stateEstimatorData.result->vWorld;


}
