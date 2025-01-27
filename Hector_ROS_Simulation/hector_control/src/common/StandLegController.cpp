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
    seResult = data->_stateEstimator->getResult();
    updateFootPosition();
    
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

void standLegController::computeFootPlacement(){

    auto &stateCommand = data->_desiredStateCommand;
    Vec3<double> v_des_robot(stateCommand->data.stateDes[6], stateCommand->data.stateDes[7],0);
    Vec3<double> v_des_world;

    //standLegの着地位置の計算
    for(int foot = 0; foot < nLegs; foot++){
        // if(swingStates[foot] > 0){

            //希望胴体速度
            v_des_world = seResult.rBody.transpose() * v_des_robot; 
            // footSwingTrajectory[foot].setHeight(footHeight);               //足上げ高さ設定

            //swingLeg着地時の予想胴体位置
            Vec3<double> Pf;
            // Vec3<double> Pf = seResult.position + seResult.rBody.transpose() * (data->_biped->getHipYawLocation(foot)) + seResult.vWorld * swingTimes[foot];    // velocity * time = position   --->なのでマイナス値はおかしい
            // std::cout << "swingStates[" << foot << "] " << swingStates[foot] << " " << swingTimes[foot] << std::endl;

            //胴体倒れ込みに対する補正値
            double p_rel_max =  0.3;
            //original
            // double pfx_rel   =  1.75 * seResult.vWorld[0] * 0.5 * gait->_stance * _dtSwing +
                                // 0.1  * (seResult.vWorld[0] - v_des_world[0]);
            // double pfy_rel   =  1.75 * seResult.vWorld[1] * 0.5 * gait->_stance * _dtSwing +
                                // 0.1  * (seResult.vWorld[1] - v_des_world[1]);

            double pfx_rel, pfy_rel;
// #ifdef _HECTOR_
//             double pfx_rel   =  0.45 * seResult.vWorld[0] * 0.5 * gait->_stance * _dtSwing +
//                                 0.12  * (seResult.vWorld[0] - v_des_world[0]);

//             double pfy_rel   =  0.45 * seResult.vWorld[1] * 0.5 * gait->_stance * _dtSwing +
//                                 0.12  * (seResult.vWorld[1] - v_des_world[1]);
// #else
// #ifdef _LAMBDA_
//             double pfx_rel   =  0.45 * seResult.vWorld[0] * 0.5 * gait->_stance * _dtSwing +
//                                 0.12  * (seResult.vWorld[0] - v_des_world[0]);

//             double pfy_rel   =  0.45 * seResult.vWorld[1] * 0.5 * gait->_stance * _dtSwing +
//                                 0.12  * (seResult.vWorld[1] - v_des_world[1]);
// #else
// #ifdef _LAMBDA_R2_
//             double pfx_rel   =  0.45 * seResult.vWorld[0] * 0.5 * gait->_stance * _dtSwing +
//                                 0.12  * (seResult.vWorld[0] - v_des_world[0]);

//             double pfy_rel   =  0.45 * seResult.vWorld[1] * 0.5 * gait->_stance * _dtSwing +
//                                 0.12  * (seResult.vWorld[1] - v_des_world[1]);
// #endif
// #endif
// #endif
            pfx_rel = fminf(fmaxf(pfx_rel, -p_rel_max), p_rel_max);
            pfy_rel = fminf(fmaxf(pfy_rel, -p_rel_max), p_rel_max);

            Pf[0] += pfx_rel;
            Pf[1] += pfy_rel; 
            Pf[2] = 0.0;

            // footSwingTrajectory[foot].setFinalPosition(Pf);        

        // }

    }
}


/******************************************************************************************************/
/******************************************************************************************************/

void standLegController::computeFootDesiredPosition(){
    for(int foot = 0; foot < nLegs; foot++){
        Vec3<double> pDesFootWorld = pFoot_w[foot];
        Eigen::Vector3d hipWidthOffSet = data->_biped->getHipYawLocation(foot);
        hipWidthOffSet(2) = 0.0;
        pFoot_b[foot] = seResult.rBody * (pDesFootWorld - seResult.position) - hipWidthOffSet ;  //原点を股関節に変換
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
        if(true){   ////////////
        // if(swingStates[leg] > 0){
            computeIK(pFoot_b[leg], data->_legController->commands[leg].qDes, leg);
            // std::cout << data->_legController->commands[leg].qDes << std::endl;
            data->_legController->commands[leg].qdDes = Eigen::Matrix<double, 5, 1>::Zero();
            Eigen::VectorXd kpgains(5);
            // kpgains << 30, 30, 30, 30, 20;
            kpgains << 20, 20, 20, 20, 20;
            Eigen::VectorXd kdgains(5);
            // kdgains << 1, 1, 1, 1, 1;
            kdgains << 0.5, 0.5, 0.5, 0.5, 0.5;
             data->_legController->commands[leg].feedforwardForce << 0, 0, 0 , 0 , 0 , 0;
             data->_legController->commands[leg].pDes = pFoot_b[leg];
             data->_legController->commands[leg].vDes = vFoot_b[leg];
             data->_legController->commands[leg].kpJoint = kpgains.asDiagonal();
             data->_legController->commands[leg].kdJoint = kdgains.asDiagonal();             
             data->_legController->commands[leg].kptoe = 5; 
             data->_legController->commands[leg].kdtoe = 0.1;              
        }else{
            //Ensure no interference with stance leg controller
            Eigen::VectorXd kpgains(5);
            Eigen::VectorXd kdgains(5);
            kpgains.setZero();
            kdgains.setZero();
            data->_legController->commands[leg].kpJoint = kpgains.asDiagonal();
            data->_legController->commands[leg].kdJoint = kdgains.asDiagonal(); 
            data->_legController->commands[leg].kpCartesian = Eigen::Matrix3d::Zero();
            data->_legController->commands[leg].kdCartesian = Eigen::Matrix3d::Zero();               
        }
    }
}
