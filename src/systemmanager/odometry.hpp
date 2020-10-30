#ifndef _ODOMETRY_HPP_INCLUDED
#define _ODOMETRY_HPP_INCLUDED

#include "okapi/api.hpp"
#include "subsystem.hpp"
#include "util/struct.hpp"

class Odometry {
private:
    typedef okapi::ADIEncoder Enc;
    pros::task_t odoTask;
    util::ChassisPos data;
    util::ChassisSpeed localVel;
    
public:
    int delay = 1;
    bool taskRunning = false;
    double zeroPosA;
    bool gyroRotation = true;
    Odometry();
    void useGyroRotation();
    void useEncoderRotation();
    void reset(util::ChassisPos, bool hardware = true);
    void startTask();
    void endTask();
    void resetForAuton();
    util::ChassisPos getPos();
    util::ChassisSpeed getChassisVel();
    void setChassisVel(util::ChassisSpeed);
    void setPos(util::ChassisPos);
};
#endif /* _ODOMETRY_HPP_INCLUDED */
