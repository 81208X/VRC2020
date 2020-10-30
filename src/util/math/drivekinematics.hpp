#ifndef _DRIVEKINEMATICS_HPP_INCLUDED
#define _DRIVEKINEMATICS_HPP_INCLUDED

#include "Eigen/Core"
#include "Eigen/QR"
#include "util/util.hpp"

namespace util {
    class InverseKinematics {
    public:
        InverseKinematics(Pos2d);

        /**
         * Calculates the wheel speed from an input chassis speed using inverse kinematics.
         * 
         * \param chassisSpeed the target chassis speed
         * 
         * \param COR the center of rotation relative to the center of the robot. Defaults to one that's previously set.
         */
        WheelSpeed toWheelSpeed(ChassisSpeed, Pos2d = Pos2d());
        bool initialized = false;
        
    private:
        /* The positions for the four wheel, relative to the center of the robot */

        /// relative position of the left front wheel
        Pos2d lf;

        /// relative position of the right front wheel
        Pos2d rf;
        
        /// relative position of the left rear wheel
        Pos2d lr;

        /// relative position of the right rear wheel
        Pos2d rr;

        /// Matrix used to do the inverse kinematics calculation from chassis speed to wheel speed.
        Eigen::Matrix<double, 4, 3> inverseKinematics;

        /// The Center Of Rotation relative to the center of the robot
        Pos2d COR;

        /// Updates the inverse kinematics matrix to match a new center of rotation
        void updateInverseKinematics(Pos2d Cor);
    };
}
#endif /* _DRIVEKINEMATICS_HPP_INCLUDED */
