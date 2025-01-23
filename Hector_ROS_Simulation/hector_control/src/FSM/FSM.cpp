#include "../../include/FSM/FSM.h"
#include <iostream>

FSM::FSM(ControlFSMData *data)
    :_data(data)
{
    _stateList.invalid = nullptr;
    _stateList.passive = new FSMState_Passive(_data);
    _stateList.walking = new FSMState_Walking(_data);
    _stateList.standing = new FSMState_Standing(_data);
    _stateList.TO = new FSMState_TO(_data);

    initialize();
}

FSM::~FSM(){
    _stateList.deletePtr();
}

void FSM::initialize()
{
    std::cout << "FSM::initialize()" << std::endl;
    
    count = 0;
    // _currentState = _stateList.walking;
    _currentState = _stateList.standing;
    _currentState -> enter();
    _nextState = _currentState;
    _mode = FSMMode::NORMAL;
}

void FSM::run()
{
    std::cout << "FSM::run()" << std::endl;

    std::cout << "======== currentState Name " << _currentState->_stateNameStr << std::endl;

    if(!checkSafty())
    {
        if(_currentState->_stateName != FSMStateName::PDSTAND)
            _data->_interface->setPassive();
    }

    if(_mode == FSMMode::NORMAL)
    {
        _currentState->run();
        _nextStateName = _currentState->checkTransition();
        if(_nextStateName != _currentState->_stateName)
        {
            _mode = FSMMode::CHANGE;
            _nextState = getNextState(_nextStateName);
        }
    }
    else if(_mode == FSMMode::CHANGE)
    {
        _currentState->exit();
        _currentState = _nextState;
        _currentState->enter();
        _mode = FSMMode::NORMAL;
        _currentState->run();       
    }

    count++;
}

FSMState* FSM::getNextState(FSMStateName stateName)
{
    std::cout << "FSM::getNextState()" << std::endl;
    
    switch(stateName)
    {
        case FSMStateName::INVALID:
            return _stateList.invalid;
        break;
        case FSMStateName::PASSIVE:
            return _stateList.passive;
        break;
        case FSMStateName::WALKING:
            return _stateList.walking;
        break;
        case FSMStateName::PDSTAND:
            return _stateList.standing;
        break;
        default:
            return _stateList.invalid;
        break;
    }
}

bool FSM::checkSafty()
{
    std::cout << "FSM::checkSafty()" << std::endl;

    if(_data->_stateEstimator->getResult().rBody(2,2) < 0.5)
    {
        return false;
    }
    else
    {
        return true;
    }
}