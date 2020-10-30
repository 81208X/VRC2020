#ifndef _AUTO_HPP_INCLUDED
#define _AUTO_HPP_INCLUDED

#include "api.h"
#include "okapi/api.hpp"

#include "util/struct.hpp"

// void follow_path(std::vector<Pos2d> &inPathToFollow, double inOverallSpeed, double inPowerfactor, double inTurningfactor, double indTolerance, double inaTolerance, double inaccelmilli, double inMaxLookAhead, double inMinLookAhead, double inlookAheadIncreaseDistance, bool inStopAtEnd);

class AutoDrive {
private:
    pros::task_t autoTask; //The task handling the automatic driving logic
    const double speedDefault = 1; //The default speed
    const double turningSpeedDefault = 1;//The default turn speed
    const double toleranceDefault = 1.5;//The default tolerance
    const bool isStopAtEndDefault = true;//The default setting for wether or not the robot should stop after an automatic movement
    const bool absLimitDefault = 1; //The default maximum percentage speed for a motor to spin at
    const double angleToleranceDefault = d2r(3.5); //The default heading tolerance in radians

    //The default distance when the robot gives up on turning
    //This value helps in preventing the robot from spinning in circles from missing its target slightly
    const double strafeDistanceDefault = 3;

    const int timeoutDefault = 5000; //The default timeout in milliseconds

public:
    AutoDrive();
    
    /**
     * Have the robot drive to a point on the field automatically.
     * \param target The coordinate of the target point.
     * \return Returns true as the rob
     */
    bool driveToPointAsync(const util::Pos2d input);
    bool turnToAngleAsync(double targetAngle);

    /**
     * The Flag is used internally to determine what the robot is doing.
     * 0 = not moving automatically
     * 1 = moving to a point automatically
     * 2 = turning to an angle automatically
     */
    enum AutoFlag {
        IDLE = 0,
        DRIVING_TO_POINT = 1,
        TURNING = 2
    };

    AutoFlag flag = IDLE;
    
    /// Stops any automatic movements, if any.
    void stop();
    /// Sets the tolerance in distance in inches. Returns a refrence to this object so you can chain functions.
    AutoDrive& withTolerance(const double input);
    ///Sets the maximum speed in perecentage. Returns a refrence to this object so you can chain functions.
    AutoDrive& withSpeed(double input);
    /// Sets the maximum turning speed in perecentage. Returns a refrence to this object so you can chain functions.
    AutoDrive& withTurnSpeed(double input);
    /// Sets wether or not the robot should stop after an automatic movement. Returns a refrence to this object so you can chain functions.
    AutoDrive& stopAtEnd(bool input);
    /// Sets the maximum speed for any motor to spin at in percentage. Returns a refrence to this object so you can chain functions.
    AutoDrive& withMaxMotorSpeed(double speed);
    /// Sets the tolerance on the robot's heading. Returns a refrence to this object so you can chain functions.
    AutoDrive& withAngleTolerance(double input);
    /// Sets the distance at which the robot gives up on turning. Returns a refrence to this object so you can chain functions.
    AutoDrive& withStrafeDistance(double input);
    ///  Sets the timeout for an automatic movement. Returns a refrence to this object so you can chain functions.
    AutoDrive& withTimeout(int input);
    /// Resets all configs to default. Returns a refrence to this object so you can chain functions.
    AutoDrive& resetSettings();
    /**
     * Checks if the automatic movement has finished.
     * 
     * \return True if flag is 0, which generally means that the automatic movement have finished,
     * false otherwise.
     */
    bool isSettled();
};
#endif /* _AUTO_HPP_INCLUDED */
