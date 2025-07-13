#ifndef STANDLEGCONTROLLER_H
#define STANDLEGCONTROLLER_H
// #include "../../ConvexMPC/GaitGenerator.h"
#include "../../include/common/ControlFSMData.h"
#include "../../include/common/FootSwingTrajectory.h"
#include "../../include/common/cppTypes.h"
#include "../../include/common/LegController.h"
#include "../../include/common/LegIk.h"
#include "./robot_select.h"


 /**
  * @note varibles with _w are in world frame
  * @note varibles with _b are in body frame
 */
class standLegController : public LegIk {
    public:
        static constexpr int nLegs = 2;
    
        standLegController() = default;
        ~standLegController() = default;

        standLegController(ControlFSMData *data, double dtSwing);

        /**
         * @brief Initialize the swing leg controller
         * @param data: pointer to the control data
         * @param dtSwing: time step for the swing leg controller
         * @note This function is an alternative to the constructor in case 
         *       the gait generator and control data are not available at 
         *       the time of construction
        */
        void initStandLegController(ControlFSMData *data, double dtSwing);
        
        /**
         * @brief Update the swing leg controller
         * @note This function should be called at every control loop iteration
         */
        void updateStandLeg();
        
        /**
         * @brief Compute an approximate inverse kinematics for 5-DoF swing leg
         * @param bodyPositionDesired: desired position of the end effector in the body frame
         * @param leg: leg index (0 for left, 1 for right)  
         * @param jointAngles: output joint angles
        */
        void computeIK(const Vec3<double> &bodyPositionDesired, Eigen::Matrix<double, 5, 1> &jointAngles, int leg);
 
        

    private:
        const ControlFSMData* data;
        StateEstimate seResult;
        double _dtSwing;
        Vec3<double> pFoot_w[nLegs];
        Vec3<double> pFoot_b[nLegs];
        Vec3<double> vFoot_b[nLegs];                        
        Vec5<double> qDes[nLegs];  
        bool firstSwing[nLegs] = {true, true};        
        
        
        void updateFootPosition();
        void computeFootPlacement();
        void computeFootDesiredPosition();
        void setDesiredJointState();
        Vec5<double> kpgains;
        Vec5<double> kdgains;

    public:
        void updateState();



        // constants can be adjusted if needed
        // const double _dt = 0.001;

}; // class standLegController

#endif // STANDLEGCONTROLLER_H    