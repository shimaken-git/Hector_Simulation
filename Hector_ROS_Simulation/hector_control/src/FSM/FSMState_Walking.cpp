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

    // _data->_stateEstimator->init_p_world();          //FSMState_Standに移行
    // _data->_stateEstimator->set_firstStage(true);    //FSMState_Standに移行
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

