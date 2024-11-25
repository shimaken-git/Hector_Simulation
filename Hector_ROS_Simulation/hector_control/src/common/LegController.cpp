#include "../../include/common/LegController.h"
#include <eigen3/Eigen/Core>
#include "../../include/common/robot_select.h"

// upper level of joint controller 
// send data to joint controller

Vec3<double> computeLegPosition(Vec5<double>& q, int leg, Mat65<double> *jfm, Mat35<double> *jf);
Vec3<double> rot2omega(Eigen::Matrix3d r);


void LegControllerCommand::zero(){
    tau = Vec5<double>::Zero();
    qDes = Vec5<double>::Zero();
    qdDes = Vec5<double>::Zero();
    pDes = Vec3<double>::Zero();
    vDes = Vec3<double>::Zero();
    feedforwardForce = Vec6<double>::Zero();
    hiptoeforce = Vec3<double>::Zero();
    kpCartesian = Mat3<double>::Zero(); 
    kdCartesian = Mat3<double>::Zero();
    kpJoint = Mat5<double>::Zero();
    kdJoint = Mat5<double>::Zero();
    double kptoe = 0;
    double kdtoe = 0;
}

/*
 * Zero leg data
 */ 
void LegControllerData::zero(){
    q = Vec5<double>::Zero();
    qd = Vec5<double>::Zero();
    p = Vec3<double>::Zero();
    v = Vec3<double>::Zero();
    J_force_moment = Mat65<double>::Zero();
    J_force = Mat35<double>::Zero();
    tau = Vec5<double>::Zero();
}

void LegController::zeroCommand(){
    for (int i = 0; i < 2; i++){
        commands[i].zero();
    }
}

void LegController::updateData(const LowlevelState* state){
    for (int leg = 0; leg < 2; leg++){
        for(int j = 0; j < 5; j++){
            data[leg].q(j) = state->motorState[leg*5+j].q;
            data[leg].qd(j) = state->motorState[leg*5+j].dq;
            data[leg].tau(j) = state->motorState[leg*5+j].tauEst;
            // std::cout << "motor joint data" << leg*5+j << ": "<< data[leg].q(j) << std::endl;
        }

        computeLegJacobianAndPosition(_biped, data[leg].q, &(data[leg].J_force_moment), &(data[leg].J_force), &(data[leg].p), leg);
        computeHeelToePosition(_biped, data[leg].q, &(data[leg].tip[0]), &(data[leg].tip[1]), leg);
        data[leg].v = data[leg].J_force * data[leg].qd;
    }

}

void LegController::updateCommand(LowlevelCmd* cmd){

    for (int i = 0; i < 2; i++){
        Vec6<double> footForce = commands[i].feedforwardForce;
        Vec5<double> legtau = data[i].J_force_moment.transpose() * footForce; // force moment from stance leg

        std::cout << "leg:" << i << " tau: " << legtau[0] << " " << legtau[1] << " " << legtau[2] << " " << legtau[3] << " " << legtau[4] << std::endl;
        // outputfile << "leg:" << i << " tau: " << legtau[0] << " " << legtau[1] << " " << legtau[2] << " " << legtau[3] << " " << legtau[4] << std::endl;


        // // cartesian PD control for swing foot
        // if(commands[i].kpCartesian(0,0) != 0 || commands[i].kdCartesian(0,0) != 0)
        // {
        //     Vec3<double> footForce_3d = commands[i].kpCartesian * (commands[i].pDes - data[i].p) +
        //                                 commands[i].kdCartesian * (commands[i].vDes - data[i].v);
          
        //     Vec5<double> swingtau = data[i].J_force.transpose() * footForce_3d ;

        //     // maintain hip angle tracking
        //     double kphip1 = 15;
        //     double kdhip1 = 1;
        //     swingtau(0) = kphip1*(0-data[i].q(0)) + kdhip1*(0-data[i].qd(0));
        //     // make sure foot is parallel with the ground
        //     swingtau(4) = commands[i].kptoe * (-data[i].q(3)-data[i].q(2)-data[i].q(4))+commands[i].kdtoe*(0-data[i].qd(4));

        //     for(int j = 0; j < 5; j++)
        //     {
        //         legtau(j) += swingtau(j);
        //     }
        // }

        commands[i].tau += legtau;

        for (int j = 0; j < 5; j++){
            cmd->motorCmd[i*5+j].tau = commands[i].tau(j);
            cmd->motorCmd[i*5+j].q = commands[i].qDes(j);
            cmd->motorCmd[i*5+j].dq = commands[i].qdDes(j);
            cmd->motorCmd[i*5+j].Kp = commands[i].kpJoint(j,j);
            cmd->motorCmd[i*5+j].Kd = commands[i].kdJoint(j,j);
            // std::cout << Side[i] << " " << limbName[j] <<" torque cmd  =  " << cmd->motorCmd[i*5+j].tau << std::endl;            
        }

        commands[i].tau << 0, 0, 0, 0, 0; // zero torque command to prevent interference
        
        
   
    }
    //std::cout << "cmd sent" << std::endl;
   
}

void computeLegJacobianAndPosition(Biped& _biped, Vec5<double>& q, Mat65<double>* J_f_m, Mat35<double>* J_f, 
                                       Vec3<double>* p, int leg)
{
    //J_base
#if defined(_HECTOR_)
    double l1 = 0.0705;  //z
    double l2 = 0.0135;  //x
    double l3 = 0.015;   //y
    double l4 = 0.018;   //y
    double l5 = 0.22;    //z
    double l6 = 0.22;    //z
    double l7 = 0.04;    //z
    double l9 = 0.047;   //y    //J0原点なので使わない
#else
#if defined(_LAMBDA_)
    //J0を原点とする座標計算
    double l1 = 0.0;     //
    double l2 = 0.0;
    double l3 = 0.0;
    double l4 = 0.0;
    double l5 = 0.153;
    double l6 = 0.153;
    double l7 = 0.04;
    double l9 = 0.053;   //y    //J0原点なので使わない

#else
#if defined(_LAMBDA_R2_)
    //J0を原点とする座標計算
    double l1 = 0.0;     //
    double l2 = 0.0;
    double l3 = 0.007;
    double l4 = 0.0;
    double l5 = 0.153;
    double l6 = 0.153;
    double l7 = 0.04;
    double l9 = 0.053;   //y    //J0原点なので使わない

#endif
#endif
#endif

    q(2) = q(2) + 0.3*3.14159;   //ここでいじったqはあとでも使うらしい。なんか変な実装。。。
    q(3) = q(3) - 0.6*3.14159;
    q(4) = q(4) + 0.3*3.14159;

    double q0 = q(0);
    double q1 = q(1);
    double q2 = q(2);
    double q3 = q(3);
    double q4 = q(4);

        double side = -1.0; // 1 for Left legs; -1 for right legs
    if (leg == 0){
        // std::cout<< "Leg Sign checked" << std::endl;
        side = 1.0;
    }
    // std::cout<< "Leg Sign" << side << std::endl;
    
    // Mat65<double> jfm;
    // Mat35<double> jf;
    // Vec3<double> _p = computeLegPosition(q, leg, &jfm, &jf);

    //HectorとLambdaのjacobi計算は一致したと思うが、念の為Lambda用の式を残している。
    if(J_f_m){
#if defined(_HECTOR_)
    // J_f_m->operator()(0, 0) =  sin(q0)*(l7*sin(q2 + q3 + q4) + l6*sin(q2 + q3) + l5*sin(q2) + l2) + cos(q0)*(l8*side + cos(q1)*(l3*side +  l4*side) - sin(q1)*(l7*cos(q2 + q3 + q4) + l6*cos(q2 + q3) + l5*cos(q2)));
    // J_f_m->operator()(1, 0) =  sin(q0)*(l8*side + cos(q1)*(l3*side +  l4*side) - sin(q1)*(l7*cos(q2 + q3 + q4) + l6*cos(q2 + q3) + l5*cos(q2))) - cos(q0)*(l7*sin(q2 + q3 + q4) + l6*sin(q2 + q3) + l5*sin(q2) + l2);
    J_f_m->operator()(0, 0) =  sin(q0)*(l7*sin(q2 + q3 + q4) + l6*sin(q2 + q3) + l5*sin(q2) + l2) + cos(q0)*(-l3*side + sin(q1)*l4*side - sin(q1)*(l7*cos(q2 + q3 + q4) + l6*cos(q2 + q3) + l5*cos(q2)));
    J_f_m->operator()(1, 0) =  sin(q0)*(-l3*side + sin(q1)*l4*side - sin(q1)*(l7*cos(q2 + q3 + q4) + l6*cos(q2 + q3) + l5*cos(q2))) + cos(q0)*(l7*sin(q2 + q3 + q4) + l6*sin(q2 + q3) + l5*sin(q2) - l2);
    J_f_m->operator()(2, 0) =  0.0;
    J_f_m->operator()(3, 0) = 0.0;
    J_f_m->operator()(4, 0) = 0.0;
    J_f_m->operator()(5, 0) = 1.0;

    J_f_m->operator()(0, 1) =  -sin(q0)*(sin(q1)*(l3*side +  l4*side) + cos(q1)*(l7*cos(q2 + q3 + q4) + l6*cos(q2 + q3) + l5*cos(q2)));
    J_f_m->operator()(1, 1) =  cos(q0)*(sin(q1)*(l3*side +  l4*side) + cos(q1)*(l7*cos(q2 + q3 + q4) + l6*cos(q2 + q3) + l7*cos(q2)));
    // J_f_m->operator()(2, 1) =  sin(q1)*(l7*cos(q2 + q3 + q4) + l6*cos(q2 + q3) + l5*cos(q2)) - cos(q1)*(l4*side);
    J_f_m->operator()(2, 1) =  sin(q1)*(l7*cos(q2 + q3 + q4) + l6*cos(q2 + q3) + l5*cos(q2) - l4*side);
    J_f_m->operator()(3, 1) = cos(q0);
    J_f_m->operator()(4, 1) = sin(q0);
    J_f_m->operator()(5, 1) = 0.0;

    J_f_m->operator()(0, 2) =  sin(q0)*sin(q1)*(l7*sin(q2 + q3 + q4) + l6*sin(q2 + q3) + l5*sin(q2)) - cos(q0)*(l7*cos(q2 + q3 + q4) + l6*cos(q2 + q3) + l5*cos(q2));
    J_f_m->operator()(1, 2) =  -sin(q0)*(l7*cos(q2 + q3 + q4) + l6*cos(q2 + q3) + l5*cos(q2)) - cos(q0)*sin(q1)*(l7*sin(q2 + q3 + q4) +l6*sin(q2 + q3) + l5*sin(q2));
    J_f_m->operator()(2, 2) =  cos(q1)*(l7*sin(q2 + q3 + q4) + l6*sin(q2 + q3) + l5*sin(q2));
    J_f_m->operator()(3, 2) = -cos(q1)*sin(q0);
    J_f_m->operator()(4, 2) = cos(q0)*cos(q1);
    J_f_m->operator()(5, 2) = sin(q1);

    J_f_m->operator()(0, 3) =  sin(q0)*sin(q1)*(l7*sin(q2 + q3 + q4) + l6*sin(q2 + q3)) - cos(q0)*(l7*cos(q2 + q3 + q4) + l6*cos(q2 + q3));
    J_f_m->operator()(1, 3) =  - sin(q0)*(l7*cos(q2 + q3 + q4) + l6*cos(q2 + q3)) - cos(q0)*sin(q1)*(l7*sin(q2 + q3 + q4) + l6*sin(q2 + q3));
    J_f_m->operator()(2, 3) =  cos(q1)*(l7*sin(q2 + q3 + q4) + l6*sin(q2 + q3));
    J_f_m->operator()(3, 3) = -cos(q1)*sin(q0);
    J_f_m->operator()(4, 3) = cos(q0)*cos(q1);
    J_f_m->operator()(5, 3) = sin(q1);

    J_f_m->operator()(0, 4) =  l7*sin(q2 + q3 + q4)*sin(q0)*sin(q1) - l7*cos(q2 + q3 + q4)*cos(q0);
    J_f_m->operator()(1, 4) =  - l7*cos(q2 + q3 + q4)*sin(q0) - l7*sin(q2 + q3 + q4)*cos(q0)*sin(q1);
    J_f_m->operator()(2, 4) =  l7*sin(q2 + q3 + q4)*cos(q1);
    J_f_m->operator()(3, 4) = -cos(q1)*sin(q0);
    J_f_m->operator()(4, 4) = cos(q0)*cos(q1);
    J_f_m->operator()(5, 4) = sin(q1);
#else
#if defined(_LAMBDA_) || defined(_LAMBDA_R2_)
    J_f_m->operator()(0, 0) = l5*(sin(q0)*sin(q2) - sin(q1)*cos(q0)*cos(q2)) + l6*(sin(q0)*sin(q2 + q3) - sin(q1)*cos(q0)*cos(q2 + q3)) + l7*(sin(q0)*sin(q2 + q3 + q4) - sin(q1)*cos(q0)*cos(q2 + q3 + q4));
    J_f_m->operator()(1, 0) = -l5*sin(q0)*sin(q1)*cos(q2) - l5*sin(q2)*cos(q0) - l6*sin(q0)*sin(q1)*cos(q2 + q3) - l6*sin(q2 + q3)*cos(q0) - l7*sin(q0)*sin(q1)*cos(q2 + q3 + q4) - l7*sin(q2 + q3 + q4)*cos(q0);
    J_f_m->operator()(2, 0) =  0.0;
    J_f_m->operator()(3, 0) = 0.0;
    J_f_m->operator()(4, 0) = 0.0;
    J_f_m->operator()(5, 0) = 1.0;

    J_f_m->operator()(0, 1) = -(l5*cos(q2) + l6*cos(q2 + q3) + l7*cos(q2 + q3 + q4))*sin(q0)*cos(q1);
    J_f_m->operator()(1, 1) = (l5*cos(q2) + l6*cos(q2 + q3) + l7*cos(q2 + q3 + q4))*cos(q0)*cos(q1);
    J_f_m->operator()(2, 1) = (l5*cos(q2) + l6*cos(q2 + q3) + l7*cos(q2 + q3 + q4))*sin(q1);
    J_f_m->operator()(3, 1) = cos(q0);
    J_f_m->operator()(4, 1) = sin(q0);
    J_f_m->operator()(5, 1) = 0.0;

    J_f_m->operator()(0, 2) = l5*(sin(q0)*sin(q1)*sin(q2) - cos(q0)*cos(q2)) + l6*(sin(q0)*sin(q1)*sin(q2 + q3) - cos(q0)*cos(q2 + q3)) + l7*(sin(q0)*sin(q1)*sin(q2 + q3 + q4) - cos(q0)*cos(q2 + q3 + q4));
    J_f_m->operator()(1, 2) = -l5*sin(q0)*cos(q2) - l5*sin(q1)*sin(q2)*cos(q0) - l6*sin(q0)*cos(q2 + q3) - l6*sin(q1)*sin(q2 + q3)*cos(q0) - l7*sin(q0)*cos(q2 + q3 + q4) - l7*sin(q1)*sin(q2 + q3 + q4)*cos(q0);
    J_f_m->operator()(2, 2) = (l5*sin(q2) + l6*sin(q2 + q3) + l7*sin(q2 + q3 + q4))*cos(q1);
    J_f_m->operator()(3, 2) = -cos(q1)*sin(q0);
    J_f_m->operator()(4, 2) = cos(q0)*cos(q1);
    J_f_m->operator()(5, 2) = sin(q1);

    J_f_m->operator()(0, 3) = l6*(sin(q0)*sin(q1)*sin(q2 + q3) - cos(q0)*cos(q2 + q3)) + l7*(sin(q0)*sin(q1)*sin(q2 + q3 + q4) - cos(q0)*cos(q2 + q3 + q4));
    J_f_m->operator()(1, 3) = -l6*sin(q0)*cos(q2 + q3) - l6*sin(q1)*sin(q2 + q3)*cos(q0) - l7*sin(q0)*cos(q2 + q3 + q4) - l7*sin(q1)*sin(q2 + q3 + q4)*cos(q0);
    J_f_m->operator()(2, 3) = (l6*sin(q2 + q3) + l7*sin(q2 + q3 + q4))*cos(q1);
    J_f_m->operator()(3, 3) = -cos(q1)*sin(q0);
    J_f_m->operator()(4, 3) = cos(q0)*cos(q1);
    J_f_m->operator()(5, 3) = sin(q1);

    J_f_m->operator()(0, 4) = l7*(sin(q0)*sin(q1)*sin(q2 + q3 + q4) - cos(q0)*cos(q2 + q3 + q4));
    J_f_m->operator()(1, 4) = -l7*(sin(q0)*cos(q2 + q3 + q4) + sin(q1)*sin(q2 + q3 + q4)*cos(q0));
    J_f_m->operator()(2, 4) = l7*sin(q2 + q3 + q4)*cos(q1);
    J_f_m->operator()(3, 4) = -cos(q1)*sin(q0);
    J_f_m->operator()(4, 4) = cos(q0)*cos(q1);
    J_f_m->operator()(5, 4) = sin(q1);
#endif
#endif
   }

   if(J_f){
#if defined(_HECTOR_)
    // J_f->operator()(0, 0) =  sin(q0)*(l7*sin(q2 + q3 + q4) + l6*sin(q2 + q3) + l5*sin(q2) + l2) + cos(q0)*(l8*side + cos(q1)*(l3*side +  l4*side) - sin(q1)*(l7*cos(q2 + q3 + q4) + l6*cos(q2 + q3) + l5*cos(q2)));
    // J_f->operator()(1, 0) =  sin(q0)*(l8*side + cos(q1)*(l3*side +  l4*side) - sin(q1)*(l7*cos(q2 + q3 + q4) + l6*cos(q2 + q3) + l5*cos(q2))) - cos(q0)*(l7*sin(q2 + q3 + q4) + l6*sin(q2 + q3) + l5*sin(q2) + l2);
    J_f->operator()(0, 0) =  sin(q0)*(l7*sin(q2 + q3 + q4) + l6*sin(q2 + q3) + l5*sin(q2) + l2) + cos(q0)*(-l3*side + sin(q1)*l4*side - sin(q1)*(l7*cos(q2 + q3 + q4) + l6*cos(q2 + q3) + l5*cos(q2)));
    J_f->operator()(1, 0) =  sin(q0)*(-l3*side + sin(q1)*l4*side - sin(q1)*(l7*cos(q2 + q3 + q4) + l6*cos(q2 + q3) + l5*cos(q2))) + cos(q0)*(l7*sin(q2 + q3 + q4) + l6*sin(q2 + q3) + l5*sin(q2) - l2);
    J_f->operator()(2, 0) =  0.0;

    J_f->operator()(0, 1) =  -sin(q0)*(sin(q1)*(l3*side +  l4*side) + cos(q1)*(l7*cos(q2 + q3 + q4) + l6*cos(q2 + q3) + l5*cos(q2)));
    J_f->operator()(1, 1) =  cos(q0)*(sin(q1)*(l3*side +  l4*side) + cos(q1)*(l7*cos(q2 + q3 + q4) + l6*cos(q2 + q3) + l7*cos(q2)));
    // J_f->operator()(2, 1) =  sin(q1)*(l7*cos(q2 + q3 + q4) + l6*cos(q2 + q3) + l5*cos(q2)) - cos(q1)*(l4*side);
    J_f->operator()(2, 1) =  sin(q1)*(l7*cos(q2 + q3 + q4) + l6*cos(q2 + q3) + l5*cos(q2) - l4*side);

    J_f->operator()(0, 2) =  sin(q0)*sin(q1)*(l7*sin(q2 + q3 + q4) + l6*sin(q2 + q3) + l5*sin(q2)) - cos(q0)*(l7*cos(q2 + q3 + q4) + l6*cos(q2 + q3) + l5*cos(q2));
    J_f->operator()(1, 2) =  -sin(q0)*(l7*cos(q2 + q3 + q4) + l6*cos(q2 + q3) + l5*cos(q2)) - cos(q0)*sin(q1)*(l7*sin(q2 + q3 + q4) +l6*sin(q2 + q3) + l5*sin(q2));
    J_f->operator()(2, 2) =  cos(q1)*(l7*sin(q2 + q3 + q4) + l6*sin(q2 + q3) + l5*sin(q2));

    J_f->operator()(0, 3) =  sin(q0)*sin(q1)*(l7*sin(q2 + q3 + q4) + l6*sin(q2 + q3)) - cos(q0)*(l7*cos(q2 + q3 + q4) + l6*cos(q2 + q3));
    J_f->operator()(1, 3) =  - sin(q0)*(l7*cos(q2 + q3 + q4) + l6*cos(q2 + q3)) - cos(q0)*sin(q1)*(l7*sin(q2 + q3 + q4) + l6*sin(q2 + q3));
    J_f->operator()(2, 3) =  cos(q1)*(l7*sin(q2 + q3 + q4) + l6*sin(q2 + q3));

    J_f->operator()(0, 4) =  l7*sin(q2 + q3 + q4)*sin(q0)*sin(q1) - l7*cos(q2 + q3 + q4)*cos(q0);
    J_f->operator()(1, 4) =  - l7*cos(q2 + q3 + q4)*sin(q0) - l7*sin(q2 + q3 + q4)*cos(q0)*sin(q1);
    J_f->operator()(2, 4) =  l7*sin(q2 + q3 + q4)*cos(q1);
#else
#if defined(_LAMBDA_) || defined(_LAMBDA_R2_)
    J_f->operator()(0, 0) = l5*(sin(q0)*sin(q2) - sin(q1)*cos(q0)*cos(q2)) + l6*(sin(q0)*sin(q2 + q3) - sin(q1)*cos(q0)*cos(q2 + q3)) + l7*(sin(q0)*sin(q2 + q3 + q4) - sin(q1)*cos(q0)*cos(q2 + q3 + q4));
    J_f->operator()(1, 0) = -l5*sin(q0)*sin(q1)*cos(q2) - l5*sin(q2)*cos(q0) - l6*sin(q0)*sin(q1)*cos(q2 + q3) - l6*sin(q2 + q3)*cos(q0) - l7*sin(q0)*sin(q1)*cos(q2 + q3 + q4) - l7*sin(q2 + q3 + q4)*cos(q0);
    J_f->operator()(2, 0) =  0.0;

    J_f->operator()(0, 1) = -(l5*cos(q2) + l6*cos(q2 + q3) + l7*cos(q2 + q3 + q4))*sin(q0)*cos(q1);
    J_f->operator()(1, 1) = (l5*cos(q2) + l6*cos(q2 + q3) + l7*cos(q2 + q3 + q4))*cos(q0)*cos(q1);
    J_f->operator()(2, 1) = (l5*cos(q2) + l6*cos(q2 + q3) + l7*cos(q2 + q3 + q4))*sin(q1);

    J_f->operator()(0, 2) = l5*(sin(q0)*sin(q1)*sin(q2) - cos(q0)*cos(q2)) + l6*(sin(q0)*sin(q1)*sin(q2 + q3) - cos(q0)*cos(q2 + q3)) + l7*(sin(q0)*sin(q1)*sin(q2 + q3 + q4) - cos(q0)*cos(q2 + q3 + q4));
    J_f->operator()(1, 2) = -l5*sin(q0)*cos(q2) - l5*sin(q1)*sin(q2)*cos(q0) - l6*sin(q0)*cos(q2 + q3) - l6*sin(q1)*sin(q2 + q3)*cos(q0) - l7*sin(q0)*cos(q2 + q3 + q4) - l7*sin(q1)*sin(q2 + q3 + q4)*cos(q0);
    J_f->operator()(2, 2) = (l5*sin(q2) + l6*sin(q2 + q3) + l7*sin(q2 + q3 + q4))*cos(q1);

    J_f->operator()(0, 3) = l6*(sin(q0)*sin(q1)*sin(q2 + q3) - cos(q0)*cos(q2 + q3)) + l7*(sin(q0)*sin(q1)*sin(q2 + q3 + q4) - cos(q0)*cos(q2 + q3 + q4));
    J_f->operator()(1, 3) = -l6*sin(q0)*cos(q2 + q3) - l6*sin(q1)*sin(q2 + q3)*cos(q0) - l7*sin(q0)*cos(q2 + q3 + q4) - l7*sin(q1)*sin(q2 + q3 + q4)*cos(q0);
    J_f->operator()(2, 3) = (l6*sin(q2 + q3) + l7*sin(q2 + q3 + q4))*cos(q1);

    J_f->operator()(0, 4) = l7*(sin(q0)*sin(q1)*sin(q2 + q3 + q4) - cos(q0)*cos(q2 + q3 + q4));
    J_f->operator()(1, 4) = -l7*(sin(q0)*cos(q2 + q3 + q4) + sin(q1)*sin(q2 + q3 + q4)*cos(q0));
    J_f->operator()(2, 4) = l7*sin(q2 + q3 + q4)*cos(q1);
#endif
#endif
    }

   if(p){
    p->operator()(0) = - l3*(side)*sin(q0) - l4*(side)*sin(q0)*cos(q1) - l2*cos(q0) - l7*cos(q0)*sin(q2 + q3 + q4) - l5*cos(q0)*(sin(q2) + sin(q2 + q3)) - l6*(sin(q0)*sin(q1)*(cos(q2 + q3) + cos(q2))) - l7*sin(q0)*sin(q1)*cos(q2 + q3 + q4);
    p->operator()(1) = l3*(side)*cos(q0) + l4*(side)*cos(q0)*cos(q1) - l2*sin(q0) - l7*(sin(q0)*sin(q2 + q3 + q4) - cos(q0)*sin(q1)*cos(q2 + q3 + q4)) + l6*(cos(q0)*sin(q1)*(cos(q2) + cos(q2 + q3)) - sin(q0)*(sin(q2) + sin(q2 + q3)));
    p->operator()(2) = l4*(side)*sin(q1) - cos(q1)*(l5*cos(q2) + l6*cos(q2 + q3) + l7*cos(q2 + q3 + q4)) - l1;
   }
}

void computeHeelToePosition(Biped& _biped, Vec5<double>& q, Vec3<double>* toe, Vec3<double>* heel, int leg)
{

    Vec3<double> hipyaw = _biped.getHipYawLocation(leg);
    Vec3<double> hiproll = _biped.getHipRollLocation(leg);
    double l1 = hipyaw[1];  // y
    double l2 = hipyaw[2];  // z
    double l3 = hiproll[1]; // y
    double l4 = hiproll[2]; // z
    double l5 = 0.153;      // z
    double l6 = 0.153;      // z
    double l7 = 0.04;       // z
    double l8 = 0.05;       // x
    double l9 = 0.05;       // x

    // q(2) = q(2) + 0.3*3.14159;    //重複しての変換になるのでここではいじらない
    // q(3) = q(3) - 0.6*3.14159;
    // q(4) = q(4) + 0.3*3.14159;

    double q0 = q(0);
    double q1 = q(1);
    double q2 = q(2);
    double q3 = q(3);
    double q4 = q(4);

    double x = -l3*sin(q0)*cos(q1) - l5*(sin(q0)*sin(q1)*cos(q2) + sin(q2)*cos(q0)) - l6*((-sin(q0)*sin(q1)*sin(q2) + cos(q0)*cos(q2))*sin(q3) + (sin(q0)*sin(q1)*cos(q2) + sin(q2)*cos(q0))*cos(q3)) - l7*(((-sin(q0)*sin(q1)*sin(q2) + cos(q0)*cos(q2))*sin(q3) + (sin(q0)*sin(q1)*cos(q2) + sin(q2)*cos(q0))*cos(q3))*cos(q4) + ((-sin(q0)*sin(q1)*sin(q2) + cos(q0)*cos(q2))*cos(q3) - (sin(q0)*sin(q1)*cos(q2) + sin(q2)*cos(q0))*sin(q3))*sin(q4));
    double y = l1 + l3*cos(q0)*cos(q1) - l5*(sin(q0)*sin(q2) - sin(q1)*cos(q0)*cos(q2)) - l6*((sin(q0)*sin(q2) - sin(q1)*cos(q0)*cos(q2))*cos(q3) + (sin(q0)*cos(q2) + sin(q1)*sin(q2)*cos(q0))*sin(q3)) - l7*((-(sin(q0)*sin(q2) - sin(q1)*cos(q0)*cos(q2))*sin(q3) + (sin(q0)*cos(q2) + sin(q1)*sin(q2)*cos(q0))*cos(q3))*sin(q4) + ((sin(q0)*sin(q2) - sin(q1)*cos(q0)*cos(q2))*cos(q3) + (sin(q0)*cos(q2) + sin(q1)*sin(q2)*cos(q0))*sin(q3))*cos(q4));
    double z = l2 + l3*sin(q1) - l5*cos(q1)*cos(q2) - l6*(-sin(q2)*sin(q3)*cos(q1) + cos(q1)*cos(q2)*cos(q3)) - l7*((-sin(q2)*sin(q3)*cos(q1) + cos(q1)*cos(q2)*cos(q3))*cos(q4) + (-sin(q2)*cos(q1)*cos(q3) - sin(q3)*cos(q1)*cos(q2))*sin(q4));

    toe->operator()(0) = x - l8*(sin(q0)*sin(q1)*sin(q2 + q3 + q4) - cos(q0)*cos(q2 + q3 + q4));
    toe->operator()(1) = y + l8*(sin(q0)*cos(q2 + q3 + q4) + sin(q1)*sin(q2 + q3 + q4)*cos(q0));
    toe->operator()(2) = z - l8*sin(q2 + q3 + q4)*cos(q1);

    heel->operator()(0) = x + l9*(sin(q0)*sin(q1)*sin(q2 + q3 + q4) - cos(q0)*cos(q2 + q3 + q4));
    heel->operator()(1) = y - l9*(sin(q0)*cos(q2 + q3 + q4) + sin(q1)*sin(q2 + q3 + q4)*cos(q0));
    heel->operator()(2) = z + l9*sin(q2 + q3 + q4)*cos(q1);

    // std::cout << "toe " << toe->operator()(0) << " " << toe->operator()(1) << " " << toe->operator()(2) << std::endl;
    // std::cout << "heel " << heel->operator()(0) << " " << heel->operator()(1) << " " << heel->operator()(2) << std::endl;

    // //toe
    // double toe_x = -l3*sin(q0)*cos(q1) - l5*(sin(q0)*sin(q1)*cos(q2) + sin(q2)*cos(q0)) - l6*(sin(q0)*sin(q1)*cos(q2 + q3) + sin(q2 + q3)*cos(q0)) - l7*(sin(q0)*sin(q1)*cos(q2 + q3 + q4) + sin(q2 + q3 + q4)*cos(q0)) - l8*(sin(q0)*sin(q1)*sin(q2 + q3 + q4) - cos(q0)*cos(q2 + q3 + q4));
    // double toe_y =  l1 + l3*cos(q0)*cos(q1) - l5*(sin(q0)*sin(q2) - sin(q1)*cos(q0)*cos(q2)) - l6*(sin(q0)*sin(q2 + q3) - sin(q1)*cos(q0)*cos(q2 + q3)) + l7*(-sin(q0)*sin(q2 + q3 + q4) + sin(q1)*cos(q0)*cos(q2 + q3 + q4)) + l8*(sin(q0)*cos(q2 + q3 + q4) + sin(q1)*sin(q2 + q3 + q4)*cos(q0));
    // double toe_z = l2 + l3*sin(q1) - l5*cos(q1)*cos(q2) - l6*cos(q1)*cos(q2 + q3) - l7*cos(q1)*cos(q2 + q3 + q4) - l8*sin(q2 + q3 + q4)*cos(q1);

    // //heel
    // double heel_x = -l3*sin(q0)*cos(q1) - l5*(sin(q0)*sin(q1)*cos(q2) + sin(q2)*cos(q0)) - l6*(sin(q0)*sin(q1)*cos(q2 + q3) + sin(q2 + q3)*cos(q0)) - l7*(sin(q0)*sin(q1)*cos(q2 + q3 + q4) + sin(q2 + q3 + q4)*cos(q0)) + l9*(sin(q0)*sin(q1)*sin(q2 + q3 + q4) - cos(q0)*cos(q2 + q3 + q4));
    // double heel_y = l1 + l3*cos(q0)*cos(q1) - l5*(sin(q0)*sin(q2) - sin(q1)*cos(q0)*cos(q2)) - l6*(sin(q0)*sin(q2 + q3) - sin(q1)*cos(q0)*cos(q2 + q3)) + l7*(-sin(q0)*sin(q2 + q3 + q4) + sin(q1)*cos(q0)*cos(q2 + q3 + q4)) - l9*(sin(q0)*cos(q2 + q3 + q4) + sin(q1)*sin(q2 + q3 + q4)*cos(q0));
    // double heel_z = l2 + l3*sin(q1) - l5*cos(q1)*cos(q2) - l6*cos(q1)*cos(q2 + q3) - l7*cos(q1)*cos(q2 + q3 + q4) + l9*sin(q2 + q3 + q4)*cos(q1);

    // std::cout << "    " << toe_x << " " << toe_y << " " << toe_z << std::endl;
    // std::cout << "    " << heel_x << " " << heel_y << " " << heel_z << std::endl;
}

Vec3<double> computeLegPosition(Vec5<double>& q, int leg, Mat65<double> *jfm, Mat35<double> *jf)
{
    // q(2) = q(2) + 0.3*3.14159;
    // q(3) = q(3) - 0.6*3.14159;
    // q(4) = q(4) + 0.3*3.14159;

    double q0 = q(0);
    double q1 = q(1);
    double q2 = q(2);
    double q3 = q(3);
    double q4 = q(4);

    double side = -1.0; // 1 for Left legs; -1 for right legs
    if (leg == 0){
        // std::cout<< "Leg Sign checked" << std::endl;
        side = 1.0;
    }

    //////hector
    double l1 = 0.06;
    double l2 = 0.015;
    double l3 = 0.02;
    double l4 = 0.023;
    double l5 = 0.22;
    double l6 = 0.22;
    double l7 = 0.036;
    //////wwlambda
    // double l1 = 0.0;
    // double l2 = 0.0;
    // double l3 = 0.0;
    // double l4 = 0.0;
    // double l5 = -0.152;
    // double l2 = -0.152;
    // double l3 = -0.04;

    std::vector <Vec3<double> > ax(5);
    ax[0] = Vec3<double>(0, 0, 1);
    ax[1] = Vec3<double>(1, 0, 0);
    ax[2] = Vec3<double>(0, 1, 0);
    ax[3] = Vec3<double>(0, 1, 0);
    ax[4] = Vec3<double>(0, 1, 0);

    Eigen::Matrix4d A0;
    A0 << 1, 0, 0, 0,
          0, 1, 0, 0,
          0, 0, 1, -l1,
          0, 0, 0, 1;

    Eigen::Matrix4d J0;
    J0 << cos(q0), -sin(q0), 0, 0,
          sin(q0),  cos(q0), 0, 0,
          0,        0,       1, 0,
          0, 0, 0, 1;

    Eigen::Matrix4d B0;
    B0 << 1, 0, 0, -l2,
          0, 1, 0, l3 * side,
          0, 0, 1, 0,
          0, 0, 0, 1;

    Eigen::Matrix4d J1;
    J1 << 1, 0,       0,        0,
          0, cos(q1), -sin(q1), 0,
          0, sin(q1),  cos(q1), 0,
          0,        0,       0, 1;

    Eigen::Matrix4d C0;
    C0 << 1, 0, 0, 0,
          0, 1, 0, l4 * side,
          0, 0, 1, 0,
          0, 0, 0, 1;

    Eigen::Matrix4d J2;
    J2 <<  cos(q2), 0, sin(q2), 0,
           0,       1, 0,       0,
          -sin(q2), 0, cos(q2), 0,
           0,       0, 0,       1;
           
    Eigen::Matrix4d D0;
    D0 << 1, 0, 0, 0,
          0, 1, 0, 0,
          0, 0, 1, -l5,
          0, 0, 0, 1;

    Eigen::Matrix4d J3;
    J3 <<  cos(q3), 0, sin(q3), 0,
           0,       1, 0,       0,
          -sin(q3), 0, cos(q3), 0,
           0,       0, 0,       1;
           
    Eigen::Matrix4d E0;
    E0 << 1, 0, 0, 0,
          0, 1, 0, 0,
          0, 0, 1, -l6,
          0, 0, 0, 1;

    Eigen::Matrix4d J4;
    J4 <<  cos(q4), 0, sin(q4), 0,
           0,       1, 0,       0,
          -sin(q4), 0, cos(q4), 0,
           0,       0, 0,       1;
           
    Eigen::Matrix4d F0;
    F0 << 1, 0, 0, 0,
          0, 1, 0, 0,
          0, 0, 1, -l7,
          0, 0, 0, 1;

    std::vector<Eigen::Matrix4d> mp(5);

    mp[0] = A0 * J0;
    mp[1] = mp[0] * B0 * J1;
    mp[2] = mp[1] * C0 * J2;
    mp[3] = mp[2] * D0 * J3;
    mp[4] = mp[3] * E0 * J4;
    // mp[0] = A0 * J0 * B0;
    // mp[1] = mp[0] * J1 * C0;
    // mp[2] = mp[1] * J2 * D0;
    // mp[3] = mp[2] * J3 * E0;
    // mp[4] = mp[3] * J4 * F0;
    Eigen::Matrix4d tip = mp[4] * F0;
    // Eigen::Matrix4d tip = A0 * J0 * B0 * J1 * C0 * J2 * D0 * J3 * E0 * J4 * F0;

    std::cout << "tip" << std::endl << tip << std::endl;

    Vec3<double> t = mp[4].block(0,3,3,1);
    for(int i = 0; i < 5; i++){
        Vec3<double> p = mp[i].block(0, 3, 3,1);
        Eigen::Matrix3d r = mp[i].block(0,0,3,3);
        Vec3<double> jr = (r * ax[i]).cross(t - p);
        Vec3<double> w = r * ax[i];
        jf->operator()(0,i) = jfm->operator()(0,i) = jr(0);
        jf->operator()(1,i) = jfm->operator()(1,i) = jr(1);
        jf->operator()(2,i) = jfm->operator()(2,i) = jr(2);
        jfm->operator()(3,i) = w(0);
        jfm->operator()(4,i) = w(1);
        jfm->operator()(5,i) = w(2);
    }
    std::cout << "jfm" << std::endl << *jfm << std::endl;
    std::cout << "jf" << std::endl << *jf << std::endl;

    Vec3<double> p = tip.block(0, 3, 3, 1);
    return p;
}

Vec3<double> rot2omega(Eigen::Matrix3d r)
{
    Vec3<double> el;
    Vec3<double> w;
    el << r(2,1) - r(1,2), r(0,2) - r(2,0), r(1,0) - r(0,1);
    double norm_el = el.norm();
    double norm = sqrt(el(0) * el(0) + el(1) * el(1) + el(2) * el(2));
    // std::cout << "norm " << norm_el << " " << norm << std::endl;
    if(norm_el > 0){
        w = atan2(norm_el, r.trace() - 1) / norm_el * el;
    }else if(r(0,0) > 0 && r(1,1) > 0 && r(2,2) > 0){
        w << 0, 0, 0;
    }else{
        w << r(0,0)+1, r(1,1)+1, r(2,2)+1;
        w = M_PI * w;
    }
    return w;
}
