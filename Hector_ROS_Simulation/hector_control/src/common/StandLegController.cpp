#include "../../include/common/StandLegController.h"
#include "../../include/common/Math/orientation_tools.h"
#include "../../include/common/robot_select.h"

/******************************************************************************************************/
/******************************************************************************************************/

standLegController::standLegController(ControlFSMData *data, double dtSwing){
    std::cout << "standLegController construct start." << std::endl;
    initStandLegController(data, dtSwing);
    std::cout << "standLegController construct end." << std::endl;
}

/******************************************************************************************************/
/******************************************************************************************************/

void standLegController::initStandLegController(ControlFSMData *data, double dtSwing){
    std::cout << "standLegController initialize start." << std::endl;
    this->data = data;
    _dtSwing = dtSwing;
    L_hipYawLocation = data->_biped->getHipYawLocation(0);
    L_hipRollLocation = data->_biped->getHipRollLocation(0);
    R_hipYawLocation = data->_biped->getHipYawLocation(1);
    R_hipRollLocation = data->_biped->getHipRollLocation(1);
    FootHeight = data->_biped->foot_height;
    Height = data->_biped->height;
    seResult = data->_stateEstimator->getResult();
    updateFootPosition();
#ifdef BEAR_REAL
    kpgains << 20, 30, 30, 30, 10;
    kdgains << 1.0, 1.0, 1.0, 1.0, 0.5;
#else
    kpgains << 20, 20, 20, 20, 20;     //膝の垂れを再現するためにkpgainを落とした
    kdgains << 1, 1, 1, 1, 1;
#endif
    
    std::cout << "standLegController initialize end." << std::endl;
}

/******************************************************************************************************/
/******************************************************************************************************/

void standLegController::updateState(){
    seResult = data->_stateEstimator->getResult();
    updateFootPosition();
}

void standLegController::updateStandLeg(){
    computeFootDesiredPosition();
    setDesiredJointState();
}

void standLegController::updateLegState(){
    std::cout << "updateLegState()" << std::endl;
    for(int i = 0; i < nLegs; i++){
        std::cout << "data[" << i << "].p = " <<  data->_legController->data[i].p.transpose() << std::endl;
        std::cout << "qDes: " << data->_legController->commands[i].qDes[3] << " q: " << data->_legController->data[i].q[3] << std::endl;
        double _p = ori::rotationMatrixToRPY(seResult.rBody)[1];
        std::cout << "pitch " << _p << std::endl;
        if(fabs(_p) < 0.5)
            data->_legController->commands[i].qDes[3] += _p * 0.001;
    }
}

/******************************************************************************************************/
/******************************************************************************************************/

void standLegController::updateFootPosition(){

    for(int i = 0; i < nLegs; i++){
        pFoot_w[i] =  seResult.position + seResult.rBody.transpose() * ( data->_biped->getHipYawLocation(i) + data->_legController->data[i].p); 
    }

    pFoot_w[0][2] = 0.0;
    pFoot_w[1][2] = 0.0;
}

/******************************************************************************************************/
/******************************************************************************************************/


void standLegController::computeFootDesiredPosition(){
    for(int foot = 0; foot < nLegs; foot++){
        Vec3<double> pDesFootWorld = pFoot_w[foot];
        Vec3<double> pDesPositionWrold = Vec3<double>(seResult.position[0], seResult.position[1], Height);  //胴体の理想位置を構築
        Eigen::Vector3d hipWidthOffSet = data->_biped->getHipYawLocation(foot);
        pFoot_b[foot] = seResult.rBody * (pDesFootWorld - pDesPositionWrold) - hipWidthOffSet ;  //股関節を原点とした足座標を作成
        std::cout << "pDesFootWorld " << pDesFootWorld.transpose() << std::endl;
        std::cout << "hipWidthOffSet " << hipWidthOffSet.transpose() << std::endl;
        std::cout << "seResult.position " << seResult.position.transpose() << std::endl;
        std::cout << "pFoot_b[" << foot << "] " << pFoot_b[foot].transpose() << std::endl;
        std::cout << "seResult.rBody" << std::endl << seResult.rBody << std::endl;
    }    
}

/******************************************************************************************************/
/******************************************************************************************************/


void standLegController::computeIK(const Vec3<double> &bodyPositionDesired, Eigen::Matrix<double, 5, 1> &jointAngles, int leg){
    computeIK_(bodyPositionDesired, jointAngles, leg);
    // jointAngles[4] = -data->_legController->data[leg].q(3)-data->_legController->data[leg].q(2) - ori::rotationMatrixToRPY(seResult.rBody)[1]; // q3 - q2
    // jointAngles[2] -= 0.05*M_PI;
    // jointAngles[3] += 0.1*M_PI;
    // jointAngles[4] -= 0.02*M_PI;
}

/******************************************************************************************************/
/******************************************************************************************************/

void standLegController::setDesiredJointState(){
    for(int leg = 0; leg < nLegs; leg++){
        if(true){
            computeIK(pFoot_b[leg], data->_legController->commands[leg].qDes, leg);
            // std::cout << data->_legController->commands[leg].qDes << std::endl;
            data->_legController->commands[leg].qdDes = Eigen::Matrix<double, 5, 1>::Zero();
// #ifdef BEAR_REAL
// // #ifdef TORQUE_RESTRICT
//             Eigen::VectorXd kpgains(5);
//             kpgains << 20, 20, 20, 20, 20;
//             Eigen::VectorXd kdgains(5);
//             kdgains << 1.0, 1.0, 1.0, 1.0, 1.0;
// #else
//             Eigen::VectorXd kpgains(5);
//             kpgains << 30, 30, 30, 30, 20;
//             Eigen::VectorXd kdgains(5);
//             kdgains << 1, 1, 1, 1, 1;
// #endif
            data->_legController->commands[leg].feedforwardForce << 0, 0, 0 , 0 , 0 , 0;
            data->_legController->commands[leg].pDes = pFoot_b[leg];
            data->_legController->commands[leg].vDes = vFoot_b[leg];
            data->_legController->commands[leg].kpJoint = kpgains.asDiagonal();
            data->_legController->commands[leg].kdJoint = kdgains.asDiagonal();             
            data->_legController->commands[leg].kptoe = 5; 
            data->_legController->commands[leg].kdtoe = 0.1;              
        }
    }
}
