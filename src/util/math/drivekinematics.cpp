#include "drivekinematics.hpp"
#include <iomanip>
#include "profiles.hpp"
#include "util/struct.hpp"

namespace util {
    InverseKinematics::InverseKinematics(Pos2d Cor) {
        updateInverseKinematics(Cor);
        initialized = true;
    }

    WheelSpeed InverseKinematics::toWheelSpeed(ChassisSpeed chassisSpeed, Pos2d Cor) {
        updateInverseKinematics(Cor);

        Eigen::Vector3d chassisSpeedsVector;
        chassisSpeedsVector << chassisSpeed.y, -chassisSpeed.x, -chassisSpeed.angle;

        Eigen::Matrix<double, 4, 1> wheelsMatrix =
            inverseKinematics * chassisSpeedsVector;

        WheelSpeed wheelSpeed;
        wheelSpeed.lf = wheelsMatrix(0, 0);
        wheelSpeed.rf = wheelsMatrix(1, 0);
        wheelSpeed.lr = wheelsMatrix(2, 0);
        wheelSpeed.rr = wheelsMatrix(3, 0);
        return wheelSpeed;
    }

    void InverseKinematics::updateInverseKinematics(Pos2d Cor) {
        if (COR == Cor && initialized)
            return;
        // outside of this class:
        // y = front of robot
        // x = right of robot
        COR = {Cor.y, -Cor.x};
        // inside of this class
        // x = front of robot
        // y = left of robot
        lf = Pos2d(BASE_LENGTH_IN / 2.0, BASE_WIDTH_IN / 2.0) - COR;
        rf = Pos2d(BASE_LENGTH_IN / 2.0, BASE_WIDTH_IN / -2.0) - COR;
        lr = Pos2d(BASE_LENGTH_IN / -2.0, BASE_WIDTH_IN / 2.0) - COR;
        rr = Pos2d(BASE_LENGTH_IN / -2.0, BASE_WIDTH_IN / -2.0) - COR;
        // TODO: change BASE_WIDTH_IN and LENGTH to variables instead of macros, so we
        // can put them in the config/menu
    
        inverseKinematics << 1, -1, -(lf.x + lf.y), 1, 1, rf.x - rf.y, 1, 1,
            lr.x - lr.y, 1, -1, -(rr.x + rr.y);
        inverseKinematics /= M_SQRT2;
    }
}