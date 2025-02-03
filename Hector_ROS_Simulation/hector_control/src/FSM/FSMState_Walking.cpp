#include "../../include/FSM/FSMState_Walking.h"

FSMState_Walking::FSMState_Walking(ControlFSMData *data)
                 :FSMState(data, FSMStateName::WALKING, "walking"),
                  Cmpc(0.001, 40, data->_biped->height, data->_biped->mass) {
                    std::cout << "FSMState_Walking construct end." << std::endl;
                  }

template<typename T0, typename T1, typename T2>
T1 invNormalize(const T0 value, const T1 min, const T2 max, const double minLim = -1, const double maxLim = 1){
	return (value-minLim)*(max-min)/(maxLim-minLim) + min;
}

void FSMState_Walking::enter()
{
    std::cout << "FSMState_Walking::enter()" << std::endl;
    v_des_body << 0, 0, 0;
    pitch = 0;
    roll = 0;
    req_stand = false;
     _data->_interface->zeroCmdPanel();
    counter = 0;
    _data->_desiredStateCommand->firstRun = true;
    _data->_stateEstimator->run(); 
    _data->_legController->zeroCommand();
    Cmpc.firstRun = true;

    // _data->_stateEstimator->init_p_world();
    _data->_stateEstimator->set_firstStage(true);
}

void FSMState_Walking::run()
{
    std::cout << "FSMState_Walking::run()" << std::endl;

    _data->_legController->updateData(_data->_lowState);
    tipRun();
    _data->_stateEstimator->run(); 
    _userValue = _data->_lowState->userValue;


    v_des_body[0] = (double)invNormalize(_userValue.ly, -0.75, 0.75);
    v_des_body[1] = (double)invNormalize(_userValue.rx, -0.25, 0.25);
    turn_rate = (double)invNormalize(_userValue.lx, -1.5, 1.5);
    _data->_desiredStateCommand->setStateCommands(roll, pitch, v_des_body, turn_rate);
    
    Cmpc.setGaitNum(2); // 2 for walking
    Cmpc.run(*_data);

    _data->_legController->updateCommand(_data->_lowCmd);  
}

void FSMState_Walking::exit()
{      
    std::cout << "FSMState_Walking::exit()" << std::endl;
    counter = 0; 
    _data->_interface->zeroCmdPanel();
}

FSMStateName FSMState_Walking::checkTransition()
{
    std::cout << "FSMState_Walking::checkTransition()" << std::endl;
    if(req_stand){
        Vec2<double> cs = Cmpc.getContactStates();
        if(cs(0) == 1.0 || cs(1) == 1.0) return FSMStateName::PDSTAND;
        else return FSMStateName::WALKING;
    }
    else if(_lowState->userCmd == UserCommand::L2_B){
        return FSMStateName::PASSIVE;
    }
    else if(_lowState->userCmd == UserCommand::L1_A){
        req_stand = true;
        return FSMStateName::WALKING;
    }
    else{
        return FSMStateName::WALKING;
    }
}

void FSMState_Walking::tipRun()
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
    // if(!(_data->_interface->contact[0] || _data->_interface->contact[1] || _data->_interface->contact[2] || _data->_interface->contact[3])){
    //     //すべてのtipがfloatingなら
    //     if(!result.firstStage){    //ファーストステージではないなら
    //         //floatingTimeが最も短いtipを探す
    //         //p_worldを推定確定
    //     }
    // }else{   //接地しているtipがあるなら
    //     if(result.firstStage){
    //         _data->_stateEstimator->set_firstStage(false);
    //         //接地しているtipのtipPrintを確定する
    //     }
    //     //tipをrtipにする
    //     //tipPrintが確定しているものはtipPrintからp_worldを試算
    //     //p_worldを決定（平均？)
    // }

    
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


    //初期状態ではロボットは浮いている => contact[] = false firstStage = true
    //p_worldは未確定 x=0,y=0,z=height => p_world_confirm = false
    //tipPrint_confirm[] = false
    //いずれかのtipが接地 => firstStage = false となる
    //接地したtip tipPrint[]を確定　z=0  p_world確定 x=0,y=0, z= 設置したtipのrtipから　p_world_confirm = true

    //２番目以降に接地したtip その時のp_worldからtipPrint[]を決定　zは成り行き（ゼロじゃなくてもよい）
    //既に確定しているtipのcontact[]=trueの場合、tipPrint[]は更新しない。（rtipだけ計算しておく）
    //複数のtipがcontactしている場合、contactしているtipのrtipの平均でp_worldを更新する。

    //足が浮いた時、floatingTimeの計数を開始する。tipPrint_confirm[]=falseとする。
    //firstStage==falseかつすべてのtipがcontact[]==falseの場合、最もfloatingTimeが小さいtipのrtipを使ってp_worldを決める。
    //p_worldにはv_world*floatingTimeを加える

    // for(int idx = 0; idx < 4; idx++){
    //     std::cout << "tipPrint[" << idx << "] " << _data->_stateEstimator->getResult().tipPrint[idx] << std::endl;
    // }

}