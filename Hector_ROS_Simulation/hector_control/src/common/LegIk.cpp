
#include "../../include/common/LegIk.h"


void LegIk::computeIK_(const Vec3<double> &bodyPositionDesired, Eigen::Matrix<double, 5, 1> &jointAngles, int leg){          

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
        jointAngles[4] = -jointAngles[3] - jointAngles[2];
        // jointAngles[4] = -data->_legController->data[leg].q(3)-data->_legController->data[leg].q(2); // q3 - q2        
#else
#if defined(_LAMBDA_) || defined(_LAMBDA_R2_)
        double l = 0.153;
        Eigen::Vector3d hip_roll(L_hipRollLocation[0], L_hipRollLocation[0], L_hipRollLocation[2]-0.03);
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
#ifdef HUMAN
        jointAngles[2] = -std::acos(acosArg1) - std::acos(acosArg2) * (foot_des_to_hip_roll[0]) / divisor;     //ヒト足
        jointAngles[3] = M_PI - 2.0 * std::asin(clamp(distance_2D_xOz / 2.0 / l, -1.0, 1.0));     //ヒト足
#else
        jointAngles[2] = std::acos(acosArg1) - std::acos(acosArg2) * (foot_des_to_hip_roll[0]) / divisor;   //鳥足
        jointAngles[3] = 2.0 * std::asin(clamp(distance_2D_xOz / 2.0 / l, -1.0, 1.0)) - M_PI;   //鳥足
#endif
        jointAngles[4] = -jointAngles[3] - jointAngles[2];
        // jointAngles[4] = -data->_legController->data[leg].q(3)-data->_legController->data[leg].q(2) - ori::rotationMatrixToRPY(seResult.rBody)[1]; // q3 - q2
#endif
#endif
}
