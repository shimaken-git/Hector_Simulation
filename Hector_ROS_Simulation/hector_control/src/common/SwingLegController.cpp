#include "../../include/common/SwingLegController.h"
#include "../../include/common/Math/orientation_tools.h"
#include "../../include/common/robot_select.h"

/******************************************************************************************************/
/******************************************************************************************************/

swingLegController::swingLegController(ControlFSMData *data, Gait* gait, double dtSwing){
    std::cout << "swingLegController construct start." << std::endl;
    this->data = data;
    this->gait = gait;
    _dtSwing = dtSwing;
    L_hipYawLocation = data->_biped->getHipYawLocation(0);
    L_hipRollLocation = data->_biped->getHipRollLocation(0);
    R_hipYawLocation = data->_biped->getHipYawLocation(1);
    R_hipRollLocation = data->_biped->getHipRollLocation(1);
    updateFootPosition();
    
    for(int i = 0; i < nLegs; i++){
      footSwingTrajectory[i].setHeight(0.0);
      footSwingTrajectory[i].setInitialPosition(pFoot_w[i]);
      footSwingTrajectory[i].setFinalPosition(pFoot_w[i]);
    }
    std::cout << "swingLegController construct end." << std::endl;
}

/******************************************************************************************************/
/******************************************************************************************************/

void swingLegController::initSwingLegController(ControlFSMData *data, Gait* gait, double dtSwing){
    std::cout << "swingLegController initialize start." << std::endl;
    this->data = data;
    this->gait = gait;
    _dtSwing = dtSwing;
    L_hipYawLocation = data->_biped->getHipYawLocation(0);
    L_hipRollLocation = data->_biped->getHipRollLocation(0);
    R_hipYawLocation = data->_biped->getHipYawLocation(1);
    R_hipRollLocation = data->_biped->getHipRollLocation(1);
    seResult = data->_stateEstimator->getResult();
    updateFootPosition();
    
    for(int i = 0; i < nLegs; i++){
      footSwingTrajectory[i].setHeight(0.1);
      footSwingTrajectory[i].setInitialPosition(pFoot_w[i]);
      footSwingTrajectory[i].setFinalPosition(pFoot_w[i]);
    }
    std::cout << "swingLegController initialize end." << std::endl;
}

/******************************************************************************************************/
/******************************************************************************************************/

void swingLegController::updateSwingLeg(){
    seResult = data->_stateEstimator->getResult();
    updateFootPosition();
    updateSwingStates();
    updateSwingTimes();
    computeFootPlacement();     
    computeFootDesiredPosition();
    setDesiredJointState();
}

/******************************************************************************************************/
/******************************************************************************************************/

void swingLegController::updateFootPosition(){

    for(int i = 0; i < nLegs; i++){    
        pFoot_w[i] =  seResult.position + seResult.rBody.transpose() * ( data->_biped->getHipYawLocation(i) + data->_legController->data[i].p); 
    }

    pFoot_w[0][2] = 0.0;
    pFoot_w[1][2] = 0.0;
}

/******************************************************************************************************/
/******************************************************************************************************/

void swingLegController::updateSwingStates(){
    swingStates = gait->getSwingSubPhase();
    contactStates = gait->getContactSubPhase();
}

/******************************************************************************************************/
/******************************************************************************************************/

void swingLegController::updateSwingTimes(){
    for(int leg = 0; leg < nLegs; leg++){
        if(firstSwing[leg]){
            swingTimes[leg] = _dtSwing * gait->_swing;
        }else{
            swingTimes[leg] -= _dt;
            if(contactStates[leg] > 0){
                firstSwing[leg] = true;
            }            
        }
    }
}

/******************************************************************************************************/
/******************************************************************************************************/

void swingLegController::computeFootPlacement(){

    auto &stateCommand = data->_desiredStateCommand;
    Vec3<double> v_des_robot(stateCommand->data.stateDes[6], stateCommand->data.stateDes[7],0);
    Vec3<double> v_des_world;

    //swingLegの着地位置の計算
    for(int foot = 0; foot < nLegs; foot++){
        // if(swingStates[foot] > 0){

            //希望胴体速度
            v_des_world = seResult.rBody.transpose() * v_des_robot; 
            footSwingTrajectory[foot].setHeight(footHeight);               //足上げ高さ設定

            //swingLeg着地時の予想胴体位置
            Vec3<double> Pf = seResult.position + seResult.rBody.transpose() * (data->_biped->getHipYawLocation(foot)) + seResult.vWorld * swingTimes[foot];    // velocity * time = position   --->なのでマイナス値はおかしい
            // std::cout << "swingStates[" << foot << "] " << swingStates[foot] << " " << swingTimes[foot] << std::endl;

            //胴体倒れ込みに対する補正値
            double p_rel_max =  0.3;
            //original
            // double pfx_rel   =  1.75 * seResult.vWorld[0] * 0.5 * gait->_stance * _dtSwing +
                                // 0.1  * (seResult.vWorld[0] - v_des_world[0]);
            // double pfy_rel   =  1.75 * seResult.vWorld[1] * 0.5 * gait->_stance * _dtSwing +
                                // 0.1  * (seResult.vWorld[1] - v_des_world[1]);

#ifdef _HECTOR_
            double pfx_rel   =  0.45 * seResult.vWorld[0] * 0.5 * gait->_stance * _dtSwing +
                                0.12  * (seResult.vWorld[0] - v_des_world[0]);

            double pfy_rel   =  0.45 * seResult.vWorld[1] * 0.5 * gait->_stance * _dtSwing +
                                0.12  * (seResult.vWorld[1] - v_des_world[1]);
#else
#ifdef _LAMBDA_
            double pfx_rel   =  0.45 * seResult.vWorld[0] * 0.5 * gait->_stance * _dtSwing +
                                0.12  * (seResult.vWorld[0] - v_des_world[0]);

            double pfy_rel   =  0.45 * seResult.vWorld[1] * 0.5 * gait->_stance * _dtSwing +
                                0.12  * (seResult.vWorld[1] - v_des_world[1]);
#else
#ifdef _LAMBDA_R2_
            double pfx_rel   =  0.45 * seResult.vWorld[0] * 0.5 * gait->_stance * _dtSwing +
                                0.12  * (seResult.vWorld[0] - v_des_world[0]);

            double pfy_rel   =  0.45 * seResult.vWorld[1] * 0.5 * gait->_stance * _dtSwing +
                                0.12  * (seResult.vWorld[1] - v_des_world[1]);
#endif
#endif
#endif
            pfx_rel = fminf(fmaxf(pfx_rel, -p_rel_max), p_rel_max);
            pfy_rel = fminf(fmaxf(pfy_rel, -p_rel_max), p_rel_max);

            Pf[0] += pfx_rel;
            Pf[1] += pfy_rel; 
            Pf[2] = 0.0;

            footSwingTrajectory[foot].setFinalPosition(Pf);        

        // }

    }
}


/******************************************************************************************************/
/******************************************************************************************************/

void swingLegController::computeFootDesiredPosition(){
    for(int foot = 0; foot < nLegs; foot++){
        if(swingStates[foot] > 0){
            // if (firstSwing[foot]){
            if (firstSwing[foot] && swingStates[foot] < 1.0){     // <----- swingStates[foot]==1の時はスルー 
                std::cout << "firstSwing[" << foot << "] " << firstSwing[foot] << std::endl;
                firstSwing[foot] = false;
                footSwingTrajectory[foot].setInitialPosition(pFoot_w[foot]);
            }
            //Compute and get the desired foot position and velocity
            std::cout << "swingStates[" << foot << "] " << swingStates[foot] << " swingTimes[" << foot << "] " << swingTimes[foot] << std::endl;
            footSwingTrajectory[foot].computeSwingTrajectoryBezier(swingStates[foot], 0.2); //FIX: second argument not used in function
            Vec3<double> pDesFootWorld = footSwingTrajectory[foot].getPosition().cast<double>();
            Vec3<double> vDesFootWorld = footSwingTrajectory[foot].getVelocity().cast<double>();
            // std::cout << "pDesFootWorld " << std::endl << pDesFootWorld << std::endl;
            // std::cout << "iniPos" << std::endl << footSwingTrajectory[foot].getIniPos().cast<double>() << std::endl;
            // std::cout << "finPos" << std::endl << footSwingTrajectory[foot].getFinPos().cast<double>() << std::endl;

            // double side = (foot == 1) ? 1.0 : -1.0; //Left foot (0) side = -1.0, Right foot (1) side = 1.0
            // Eigen::Vector3d hipWidthOffSet = {-0.015, side*-0.057, 0.0}; // TODO: sync with Biped.h
            Eigen::Vector3d hipWidthOffSet = data->_biped->getHipYawLocation(foot);
            hipWidthOffSet(2) = 0.0;
            pFoot_b[foot] = seResult.rBody * (pDesFootWorld - seResult.position) - hipWidthOffSet ;  //原点を股関節に変換
            // vFoot_b[foot] = seResult.rBody * (vDesFootWorld*0 - seResult.vWorld);             
            vFoot_b[foot] = seResult.rBody * (vDesFootWorld - seResult.vWorld);             
        }
    }    
}

/******************************************************************************************************/
/******************************************************************************************************/


void swingLegController::computeIK(const Vec3<double> &bodyPositionDesired, Eigen::Matrix<double, 5, 1> &jointAngles, int leg){          

        //////hector
        // double l1 = 0.06;
        // double l2 = 0.0135;
        // double l3 = 0.015;
        // double l4 = 0.018;
        // double l5 = 0.22;
        // double l6 = 0.22;
        // double l7 = 0.04;
        // double l8 = 0.015;

        Vec3<double> pFoot_des_b = bodyPositionDesired;
        double side = (leg == 0) ? -1.0 /*Left foot in swing*/ : 1.0 /*Right foot in swing*/;

#if defined(_HECTOR_)
        Eigen::Vector3d hip_roll(L_hipRollLocation[0]-0.06, 0.0, L_hipYawLocation[2]+L_hipRollLocation[2]-0.07);  // -0.06 = thigh_offset_x
                                                                                                                  // -0.07 足の高さ分かな？
        Eigen::Vector3d foot_des_to_hip_roll = pFoot_des_b - hip_roll; //in hip roll frame
        double distance_3D = foot_des_to_hip_roll.norm();
        double distance_2D_yOz = std::sqrt(std::pow(foot_des_to_hip_roll[1], 2) + std::pow(foot_des_to_hip_roll[2], 2));
        // double distance_horizontal = 0.0205;
        double distance_horizontal = 0.018;
        double distance_vertical = std::sqrt(std::max(0.00001, std::pow(distance_2D_yOz, 2) - std::pow(distance_horizontal, 2)));        // double distance_vertical = std::sqrt(std::pow(distance_2D_yOz, 2) - std::pow(distance_horizontal, 2));
        double distance_2D_xOz = pow(( pow(distance_3D,2.0)-pow(distance_horizontal,2.0)), 0.5);
                       
        // Ensure arguments are within valid range for acos and asin
        double acosArg1 = clamp(distance_2D_xOz / (2.0 * 0.22), -1.0, 1.0);
        double acosArg2 = clamp(distance_vertical / distance_2D_xOz, -1.0, 1.0);
        double divisor = std::abs(foot_des_to_hip_roll[0]);
        divisor = (divisor == 0.0) ? 1e-6 : divisor; // Prevent division by zero

        // Joint angle calculations
        jointAngles[0] = 0.0; 
        jointAngles[1] = std::asin(clamp(foot_des_to_hip_roll[1] / distance_2D_yOz, -1.0, 1.0)) + std::asin(clamp(distance_horizontal * side / distance_2D_yOz, -1.0, 1.0));        
        jointAngles[2] = std::acos(acosArg1) - std::acos(acosArg2) * (foot_des_to_hip_roll[0]) / divisor;
        jointAngles[3] = 2.0 * std::asin(clamp(distance_2D_xOz / 2.0 / 0.22, -1.0, 1.0)) - M_PI;
        jointAngles[4] = -data->_legController->data[leg].q(3)-data->_legController->data[leg].q(2); // q3 - q2        
#else
#if defined(_LAMBDA_) || defined(_LAMBDA_R2_)
        double l = 0.153;
        Eigen::Vector3d hip_roll(L_hipRollLocation[0], 0.0, L_hipYawLocation[2]+L_hipRollLocation[2]-0.03);
                                                                                                                  // -0.03 足の高さ分かな？
        Eigen::Vector3d foot_des_to_hip_roll = pFoot_des_b - hip_roll; //in hip roll frame
        double distance_3D = foot_des_to_hip_roll.norm();
        double distance_2D_yOz = std::sqrt(std::pow(foot_des_to_hip_roll[1], 2) + std::pow(foot_des_to_hip_roll[2], 2));
        // double distance_horizontal = 0.018;
        double distance_vertical = distance_2D_yOz;
        double distance_2D_xOz = distance_3D;
                       
        // Ensure arguments are within valid range for acos and asin
        double acosArg1 = clamp(distance_2D_xOz / (2.0 * l), -1.0, 1.0);
        double acosArg2 = clamp(distance_vertical / distance_2D_xOz, -1.0, 1.0);
        double divisor = std::abs(foot_des_to_hip_roll[0]);
        divisor = (divisor == 0.0) ? 1e-6 : divisor; // Prevent division by zero

        // Joint angle calculations
        jointAngles[0] = 0.0; 
        jointAngles[1] = std::asin(clamp(foot_des_to_hip_roll[1] / distance_2D_yOz, -1.0, 1.0));
        jointAngles[2] = std::acos(acosArg1) - std::acos(acosArg2) * (foot_des_to_hip_roll[0]) / divisor;
        jointAngles[3] = 2.0 * std::asin(clamp(distance_2D_xOz / 2.0 / l, -1.0, 1.0)) - M_PI;
        jointAngles[4] = -data->_legController->data[leg].q(3)-data->_legController->data[leg].q(2) - ori::rotationMatrixToRPY(seResult.rBody)[1]; // q3 - q2
#endif
#endif
        //Joint angles offset correction
        // jointAngles[2] -= 0.3*M_PI;
        // jointAngles[3] += 0.6*M_PI;
        // jointAngles[4] -= 0.3*M_PI;
        jointAngles[2] -= 0.05*M_PI;
        jointAngles[3] += 0.1*M_PI;
        jointAngles[4] -= 0.05*M_PI;
        // std::cout << "ik " << jointAngles << std::endl;
}

/******************************************************************************************************/
/******************************************************************************************************/

void swingLegController::setDesiredJointState(){
    for(int leg = 0; leg < nLegs; leg++){
        if(swingStates[leg] > 0){
            computeIK(pFoot_b[leg], data->_legController->commands[leg].qDes, leg);
            // std::cout << data->_legController->commands[leg].qDes << std::endl;
            data->_legController->commands[leg].qdDes = Eigen::Matrix<double, 5, 1>::Zero();
            Eigen::VectorXd kpgains(5);
            kpgains << 30, 30, 30, 30, 20;
            Eigen::VectorXd kdgains(5);
            kdgains << 1, 1, 1, 1, 1;
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