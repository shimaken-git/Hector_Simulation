#ifndef LEGIK_H
#define LEGIK_H

#include "../../include/common/cppTypes.h"
#include "../../include/common/Math/orientation_tools.h"
#include "../../include/common/robot_select.h"
#include<iostream>


class LegIk {
    public:
        // static constexpr int nLegs = 2;

        LegIk(){};
        ~LegIk(){};

        void computeIK_(const Vec3<double> &bodyPositionDesired, Eigen::Matrix<double, 5, 1> &jointAngles, int leg);

    protected:
        Vec3<double> L_hipYawLocation;
        Vec3<double> L_hipRollLocation;
        Vec3<double> R_hipYawLocation;
        Vec3<double> R_hipRollLocation;
        double FootHeight;

        // utility functions
        double clamp(double val, double minVal, double maxVal) {
                return std::max(minVal, std::min(val, maxVal));
        }                            

};



#endif