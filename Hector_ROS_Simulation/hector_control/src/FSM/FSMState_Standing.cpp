#include "../../include/FSM/FSMState_Standing.h"

FSMState_Standing::FSMState_Standing(ControlFSMData *data)
                 :FSMState(data, FSMStateName::PDSTAND, "standing")
                  {
                    std::cout << "FSMState_Standing construct end." << std::endl;
                  }

void FSMState_Standing::enter()
{
    std::cout << "FSMState_Standing::enter()" << std::endl;

    int horizonLength = 10;
    double dtMPC = 0.001 * 40;
    stand.initStandLegController(_data, dtMPC);

    v_des_body << 0, 0, 0;
    pitch = 0;
    roll = 0;
     _data->_interface->zeroCmdPanel();
    counter = 0;
    _data->_desiredStateCommand->firstRun = true;
    _data->_stateEstimator->run(); 
    _data->_legController->zeroCommand();  //LegControllerCommandの中身をゼロにする。feedforwardForceがクリアされるのが重要
    // Cmpc.firstRun = true;

    _data->_stateEstimator->init_p_world();
    _data->_stateEstimator->set_firstStage(true);
    first = true;
}

void FSMState_Standing::run()
{
    std::cout << "FSMState_Standing::run()" << std::endl;

    _data->_legController->updateData(_data->_lowState);
    tipRun();
    _data->_stateEstimator->run(); 
    /*　    _data->_stateEstimator->run(); で何をやっているか
    StateEstimatorContenerに登録されているEstimatorを順にrun()で更新する。
    登録されているEstimatorは
        stateEstimator->addEstimator<CheaterOrientationEstimator>();   
        stateEstimator->addEstimator<CheaterPositionVelocityEstimator>();   
    登録はmain.cppで行われる。
    */
    _userValue = _data->_lowState->userValue;
    const StateEstimate *_result = _data->_stateEstimator->getResult_();


    v_des_body[0] = 0;
    v_des_body[1] = 0;
    turn_rate = 0;
    _data->_desiredStateCommand->setStateCommands(roll, pitch, v_des_body, turn_rate);
    
    if(first){
        if(_result->position(2) != 0.0){
            stand.updateState();
            stand.updateStandLeg();
            first = false;
        }else{
            std::cout << "position" << std::endl << _result->position.transpose() << std::endl;
        }
    }else{
        stand.updateState();
    }
    stand.updateLegState();
    // stand.updateStandLeg();
    std::cout << "_result->rBody " << std::endl << _result->rBody << std::endl;
    std::cout << "_result->position " << _result->position.transpose() << std::endl;
/*  updateStandLeg()の中身
    computeFootDesiredPosition();     ==> pFoot_b[]を確定させる
        pFoo_b[]の計算に向けて
            Vec3<double> pDesFootWorld = pFoot_w[foot];
            Eigen::Vector3d hipWidthOffSet = data->_biped->getHipYawLocation(foot);
            hipWidthOffSet(2) = 0.0;
            pFoot_b[foot] = seResult.rBody * (pDesFootWorld - seResult.position) - hipWidthOffSet ;  //原点を股関節に変換
                            ↑ワールド胴体姿勢 ↑ワールド座標の足位置
                                                              ↑ワールド胴体位置   ↑股関節オフセット
    setDesiredJointState();           ==> computeIK()で関節角度を計算して、legController->commands[]を更新する。
                                          pFoot_b[]を使う。
*/
    
    _data->_legController->updateCommand(_data->_lowCmd);    //_legController->commands[] => _data->_lowCmd
}

void FSMState_Standing::exit()
{      
    std::cout << "FSMState_Standing::exit()" << std::endl;
    counter = 0; 
    _data->_interface->zeroCmdPanel();
}

FSMStateName FSMState_Standing::checkTransition()
{
    std::cout << "FSMState_Standing::checkTransition()" << std::endl;
    if(_lowState->userCmd == UserCommand::L1_X){
        return FSMStateName::WALKING;
    }
    else if(_lowState->userCmd == UserCommand::L2_B){
        return FSMStateName::PASSIVE;
    }
    else{
        return FSMStateName::PDSTAND;
    }
}
