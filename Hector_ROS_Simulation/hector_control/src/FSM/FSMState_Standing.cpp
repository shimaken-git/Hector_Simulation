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

    // _data->_stateEstimator->init_p_world();
    _data->_stateEstimator->set_firstStage(true);
    first = true;
}

void FSMState_Standing::run()
{
    std::cout << "FSMState_Standing::run()" << std::endl;

    _data->_legController->updateData(_data->_lowState);
    tipRun();
    _data->_stateEstimator->run(); 
    _userValue = _data->_lowState->userValue;
    const StateEstimate *_result = _data->_stateEstimator->getResult_();


    v_des_body[0] = 0;
    v_des_body[1] = 0;
    turn_rate = 0;
    _data->_desiredStateCommand->setStateCommands(roll, pitch, v_des_body, turn_rate);
    
    if(first){
        if(_result->position(2) != 0.0){
            stand.updateState();
            first = false;
        }else{
            std::cout << "position" << std::endl << _result->position.transpose() << std::endl;
        }
    }
    stand.updateStandLeg();
/*  updateStangLeg()の中身
    computeFootDesiredPosition();     ==> pFoot_b[]を確定させる
        pFoo_b[]の計算に向けて
            Vec3<double> pDesFootWorld = footSwingTrajectory[foot].getPosition().cast<double>();
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
    else{
        return FSMStateName::PDSTAND;
    }
}

void FSMState_Standing::tipRun()
{
    const StateEstimate *_result = _data->_stateEstimator->getResult_();
    static bool contact[4] = {false, false, false, false};
    static bool bfr_contact[4] = {false, false, false, false};
    for(int i = 0; i < 4; i++){
        bfr_contact[i] = contact[i];    //変化点をfetchできない場合があるのでこれを使う。
        contact[i] = _data->_interface->contact[i];
    }
    std::cout << "    contact " << contact[0] << " " << contact[1] << " " << contact[2] << " " << contact[3] << std::endl;
    std::cout << "bfr_contact " << bfr_contact[0] << " " << bfr_contact[1] << " " << bfr_contact[2] << " " << bfr_contact[3] << std::endl;
    
    for(int _leg = 0; _leg < 2; _leg++){
        for(int _tip = 0; _tip < 2; _tip++){
            //tipをrtipにする
            _data->_stateEstimator->calcRtip(_leg*2+_tip, _data->_legController->data[_leg].tip[_tip]);
        }
    }

    bool floating_check = true;
    bool unsettled[4] = {false,false,false,false};    // tipPrint[]が未確定のtip
    bool firstStage = _result->firstStage ? true : false;
    Vec3<double> candidate[4] = {Vec3<double>(0, 0, 0),Vec3<double>(0, 0, 0),Vec3<double>(0, 0, 0),Vec3<double>(0, 0, 0)};
    for(int i = 0; i < 4; i++){
        if(contact[i]){
            floating_check = false;
            if(_result->firstStage){
                //最初のp_world候補と、tipPrint確定
                _data->_stateEstimator->set_tipPrint(i, Vec3<double>(_result->rtip[i][0], _result->rtip[i][1], 0));
                candidate[i] = _result->tipPrint[i] - _result->rtip[i];
                std::cout << "candidate[" << i << "] " << candidate[i][0] << " " << candidate[i][1] << " " << candidate[i][2] << std::endl;
                firstStage = false;
            }else{
                if(bfr_contact[i]){
                    //前回も接地していた
                    //tipPrint[i]の更新なし
                    //p_worldの候補値を作成
                    candidate[i] = _result->tipPrint[i] - _result->rtip[i];
                    std::cout << "Deciding candidates based on previous results " << i << " " << candidate[i][0] << " " << candidate[i][1] << " " << candidate[i][2] << std::endl;
                }else{
                    //前回は浮いていたのでtipPrint[i]は未確定
                    //p_worldが決まったらtipPrint[i]を確定させる。未確定フラグを立てておく
                    unsettled[i] = true;
                    std::cout << "It was floating last time so it's not confirmed " << i << std::endl;
                }
            }
        }else{
            //tipが浮いていた
            //特にやることない？

        }
    }
    if(_result->firstStage && !firstStage) _data->_stateEstimator->set_firstStage(false);

    if(floating_check){
        //すべてのtipが浮いていたら
        if(!_result->firstStage){
            std::cout << "!!!!!!!!!!!!!!!!! All Tip Floating !!!!!!!!!!!!!!!!" << std::endl;
        }
    }else{
        //候補からp_world決定
        int n_candidates = 0;
        Vec3<double> acc_candidate(0,0,0);
        for(int i = 0; i < 4; i++){
            if(!unsettled[i] && contact[i]){
                n_candidates++;
                acc_candidate += candidate[i];
            }
        }
        std::cout << "n_candidates " << n_candidates << std::endl;
        if(n_candidates != 0){
            _data->_stateEstimator->set_p_world(acc_candidate / n_candidates);
            std::cout << "Decition p_world " << _result->p_world[0] << " " << _result->p_world[1] << " "  << _result->p_world[2] << std::endl;
        }else{
            std::cout << "Cannot be determined because there are no candidates." << std::endl;
        }
        //unsettledがあったらp_worldからtipPrint[]を決定する
        if(n_candidates != 0){
            for(int i = 0; i < 4; i++){
                if(unsettled[i]){
                    _data->_stateEstimator->set_tipPrint(i, Vec3<double>(_result->p_world[0] + _result->rtip[i][0], _result->p_world[1] + _result->rtip[i][1], 0));
                    std::cout << "The coordinates of the floating tip have been determined. " << i << " " << _result->tipPrint[i] << std::endl;
                }
            }
        }else{
                std::cout << "Cannot be the determined coordinates of floating tip because there are no candidates." << std::endl;
        }
    }
}