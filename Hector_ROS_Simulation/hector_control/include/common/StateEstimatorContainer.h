/*
MIT License

Copyright (c) 2019 MIT Biomimetics Robotics Lab

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

/*!
 * @file StateEstimator.h
 * @brief Implementation of State Estimator Interface
 *
 * Each StateEstimator object contains a number of estimators
 *
 * When the state estimator is run, it runs all estimators.
 */


#ifndef PROJECT_STATE_ESTIMATOR_CONTAINER_H
#define PROJECT_STATE_ESTIMATOR_CONTAINER_H

#pragma once

#include "LegController.h"
#include "../messages/LowlevelState.h"

/*!
 * Result of state estimation
 */

struct StateEstimate {
    Vec4<double> contactEstimate;
    Vec3<double> position;
    Vec3<double> vBody;
    Quat<double> orientation;
    Vec3<double> omegaBody;
    RotMat<double> rBody;
    Vec3<double> rpy;

    Vec3<double> omegaWorld;
    Vec3<double> vWorld;
    Vec3<double> aBody, aWorld;

    bool p_world_confirm;
    Vec3<double> p_world;         //tipPrintより算出したworld position
    Vec3<double> v_world;         //tipPrintより算出したworld velocity
    Vec3<double> rtip[4];         //tipをrBodyで回転させたもの
    Vec3<double> tipPrint[4];
    bool tipPrint_confirm[4];
    bool firstStage;              //初期のロボットが浮いている状態ならtrue
    bool tpValid[4];              //tipPrint[]が有効ならtrue
};

struct CheaterState{
  Quat<double> orientation;
  Vec3<double> position;
  Vec3<double> omegaBody;
  Vec3<double> vBody;
  Vec3<double> acceleration;
};


/*!
 * input for state estimation
 */ 
struct StateEstimatorData {
    StateEstimate* result;
    LowlevelState* lowState;
    LegControllerData* legControllerData;
};

/*!
 * All Estimators inherit from this class
 */ 
class GenericEstimator{
  public:
    virtual void run() = 0;
    virtual void setup() = 0;

    void setData(StateEstimatorData data) {_stateEstimatorData = data;};

    virtual ~GenericEstimator() = default;
    StateEstimatorData _stateEstimatorData;
};

/*!
 * Main State Estimator class
 * Contains all GenericEstimator
 */ 
class StateEstimatorContainer {
  public:
    // Constructor
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    StateEstimatorContainer(LowlevelState *_lowState,
                            LegControllerData *_legControllerData,
                            StateEstimate *stateEstimate){
        _data.lowState = _lowState;
        _data.legControllerData = _legControllerData;
        _data.result = stateEstimate;
    }
    // deconstructor
    ~StateEstimatorContainer() {
        for (auto estimator : _estimators) {
            delete estimator;
        }
    }
    // run estimator
    void run(){
        for (auto estimator : _estimators){
            estimator->run();
        }
    }

    // get result
    const StateEstimate&  getResult() {return *_data.result;}

    // get result
    const StateEstimate* getResult_() {return _data.result;}

    // update rtip
    void calcRtip(int idx, Vec3<double> tipVec){
        _data.result->rtip[idx] = _data.result->rBody.transpose() * tipVec;
    }

    // set w_positon
    void set_p_world(Vec3<double> _position){
        _data.result->p_world = _position;
    }

    // init w_positon
    void init_p_world(){
        _data.result->p_world = Vec3<double>(0, 0, _data.legControllerData->hector->height);
        _data.result->p_world_confirm = false;
        for(int i = 0; i < 4; i++) _data.result->tipPrint_confirm[i] = false;
    }

    // set tipPrint
    void set_tipPrint(int idx, Vec3<double> _p){
        _data.result->tipPrint[idx] = _p;
        _data.result->tipPrint_confirm[idx] = true;
    }

    void set_firstStage(bool b){
        _data.result->firstStage = b;
    }

    // add estimator of given type
    template <typename EstimatorToAdd>
    void addEstimator(){
        std::cout << "add estimator" << std::endl;
        auto* estimator = new EstimatorToAdd();
        estimator->setData(_data);
        estimator->setup();
        _estimators.push_back(estimator);
    }

    // remove estimator of given type
    template <typename EstimatorToRemove>
    void removeEstimator() {
        int nRemoved = 0;
        _estimators.erase(
            std::remove_if(_estimators.begin(), _estimators.end(),
                           [&nRemoved](GenericEstimator* e){
                               if (dynamic_cast<EstimatorToRemove*>(e)){
                                   delete e;
                                   nRemoved++;
                                   return true;
                                } else {
                                    return false;
                                }
                           }),
            _estimators.end());
    }

    // remove all estimators
    void removeAllEstimators() {
        for (auto estimator : _estimators) {
            delete estimator;
        }
        _estimators.clear();
    }
  private:
    std::vector<GenericEstimator*> _estimators;
    Vec4<double> _phase;
    StateEstimatorData _data;
};


#endif