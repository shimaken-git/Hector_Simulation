#ifndef PROJECT_BIPED_H
#define PROJECT_BIPED_H

#include <vector>
#include "cppTypes.h"
#include "robot_select.h"

#ifdef _HECTOR_
class Biped {
    public:
        Biped() :
            mass(13.856),
            leg_yaw_offset_x(0.0),
            leg_yaw_offset_y(0.047),
            leg_yaw_offset_z(-0.1265),
            leg_roll_offset_x(0.0465),
            leg_roll_offset_y(0.015),
            leg_roll_offset_z(-0.0705),
            hipLinkLength(0.018),
            thighLinkLength(0.22),
            calfLinkLength(0.22) {}
        Vec3<double> getHipYawLocation(int leg) const {
            checkLegIndex(leg);
            return Vec3<double>(leg_yaw_offset_x, leg == 0 ? leg_yaw_offset_y : -leg_yaw_offset_y, leg_yaw_offset_z);
        }

        Vec3<double> getHipRollLocation(int leg) const {
            checkLegIndex(leg);
            return Vec3<double>(leg_roll_offset_x, leg == 0 ? leg_roll_offset_y : -leg_roll_offset_y, leg_roll_offset_z);
        }

    private:
        void checkLegIndex(int leg) const {
            if (leg < 0 || leg >= 2) {
                throw std::invalid_argument("Invalid leg index");
            }
        }

        const double hipLinkLength, thighLinkLength, calfLinkLength;
        const double leg_yaw_offset_x, leg_yaw_offset_y, leg_yaw_offset_z;
        const double leg_roll_offset_x, leg_roll_offset_y, leg_roll_offset_z;
        const double mass;
    };
#else
#ifdef _LAMBDA_
class Biped {
    public:
        Biped() :
            mass(4.500),
            leg_yaw_offset_x(0.0),
            leg_yaw_offset_y(0.053),    // wwlambda
            // leg_yaw_offset_z(-0.042),    // CoM
            leg_yaw_offset_z(-0.091),    // CoM   +PC
            leg_roll_offset_x(0.0),
            leg_roll_offset_y(0.0),
            leg_roll_offset_z(0.0),
            hipLinkLength(0.0),
            thighLinkLength(0.153),
            calfLinkLength(0.153) {}
        Vec3<double> getHipYawLocation(int leg) const {
            checkLegIndex(leg);
            return Vec3<double>(leg_yaw_offset_x, leg == 0 ? leg_yaw_offset_y : -leg_yaw_offset_y, leg_yaw_offset_z);
        }

        Vec3<double> getHipRollLocation(int leg) const {
            checkLegIndex(leg);
            return Vec3<double>(leg_roll_offset_x, leg == 0 ? leg_roll_offset_y : -leg_roll_offset_y, leg_roll_offset_z);
        }

    private:
        void checkLegIndex(int leg) const {
            if (leg < 0 || leg >= 2) {
                throw std::invalid_argument("Invalid leg index");
            }
        }

        const double hipLinkLength, thighLinkLength, calfLinkLength;
        const double leg_yaw_offset_x, leg_yaw_offset_y, leg_yaw_offset_z;
        const double leg_roll_offset_x, leg_roll_offset_y, leg_roll_offset_z;
        const double mass;
    };
#else
#ifdef _LAMBDA_R2_
class Biped {
    public:
        Biped() :
            mass(4.500),
            leg_yaw_offset_x(0.0),
            // leg_yaw_offset_y(0.080),     // wwlambda_r2
            leg_yaw_offset_y(0.053),     // wwlambda_r2
            leg_yaw_offset_z(-0.085),    // CoM wwlambda_r2 +PC
            leg_roll_offset_x(0.0),
            leg_roll_offset_y(0.007),
            leg_roll_offset_z(0.0),
            hipLinkLength(0.0),
            thighLinkLength(0.153),
            calfLinkLength(0.153) {}
        Vec3<double> getHipYawLocation(int leg) const {
            checkLegIndex(leg);
            return Vec3<double>(leg_yaw_offset_x, leg == 0 ? leg_yaw_offset_y : -leg_yaw_offset_y, leg_yaw_offset_z);
        }

        Vec3<double> getHipRollLocation(int leg) const {
            checkLegIndex(leg);
            return Vec3<double>(leg_roll_offset_x, leg == 0 ? leg_roll_offset_y : -leg_roll_offset_y, leg_roll_offset_z);
        }

    private:
        void checkLegIndex(int leg) const {
            if (leg < 0 || leg >= 2) {
                throw std::invalid_argument("Invalid leg index");
            }
        }

        const double hipLinkLength, thighLinkLength, calfLinkLength;
        const double leg_yaw_offset_x, leg_yaw_offset_y, leg_yaw_offset_z;
        const double leg_roll_offset_x, leg_roll_offset_y, leg_roll_offset_z;
        const double mass;
    };
#endif
#endif
#endif

#endif
