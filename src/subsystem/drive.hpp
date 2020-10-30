#ifndef _DRIVE_HPP_INCLUDED
#define _DRIVE_HPP_INCLUDED

#include "util/math/drivekinematics.hpp"

#define MANUAL_DRIVE_CURVATURE 2
#define HOLD_TOLERANCE 3
#define MAXRPM 200 // This refers to real RPM, not 200 RPM simulated for gearing

class DriveSubsystem {
private:
    /// The drive curve loopup table. Should not be accessed directly. Use lookupDriveCurve() instead!
    int driveCurveLookUp[256];
    
    /**
     * Fills up the drive curve look-up table.
     * Since there are only 256 possible inputs, we pre-compute all of them to save time later on.
     * The "curve" function is descirbed here https://www.desmos.com/calculator/ntm9m2popj
     * The current version of the function on desmos has a domain and range of -1 to +1, but the math is quite similar
     * 
     * This handles both our deadzoning and drive-curve.
     * The curve's shape is designed to provide finer control when moving at medium power
     * and a wider range of control when moving at high power.
     */
    void initDriveCurveLookup();

    /**
     * Looks up the corresponding value on the drive "curve" for each input value.
     * This is made as a function because the lookup array have index 0 to +255, but our input is -127 to +127.
     * This function simply handles shifting the index to have everything match up.
     * 
     * \param input the input value from an axis on the vex controller. Should be between -127 and +127
     * 
     * \return The corresponding value on the drive "curve"
     */
    int lookupDriveCurve(int8_t input);
    util::InverseKinematics IK;
    
public:
    DriveSubsystem();
    
    /**
     * Applies a chassis speed to the robot. Calculates the correct wheel speeds using simple math.
     * The chassis speed will be normalized to have a maximum absolute value of 1.
     * 
     * \param chassisSpeed the target chassis speed relative to the robot.
     * all x, y, and rotation speeds will be in arbitrary percentage.
     * This function should only be used in driver control.
     */
    void driveSimple(util::ChassisSpeed);

    /**
     * Applies a set of wheel speeds to the robot.
     * The wheel speeds will not be normalized.
     * Make sure the absolute value of [maximum wheel speed] * [multiplier] is lower than 200
     * 
     * \param wheelSpeed the target wheel speed to apply to the drive motors.
     * 
     * \param multiplier a multiplier to be applied to the wheel speeds
     */
    void setWheelSpeed(util::WheelSpeed, double);

    /**
     * Applies a chassis speed to the robot. Calculates the correct wheel speeds using inverse kinematics.
     * The chassis speed will be normalized using the max motor speed multiplier.
     * 
     * \param chassisSpeed the target chassis speed relative to the robot.
     * x and y should be in inches per second and rotation would be in radians per second.
     * 
     * \param maxMotorSpeedMultiplier The maximum speed coefficient a motor should move at. Basically the "speed" value.
     * Can be between 0 and 1
     */
    void setChassisSpeedIK(util::ChassisSpeed cs, double maxMotorSpeedMultiplier = 1);

    ///Reacts to controller input and updates motor speeds accordingly. Should be called on every loop in opcontrol.
    void handleDriver();

    ///\return The average encoder values from the integrated encoders from the two left motors.
    double getLeftEnc();

    ///\return The average encoder values from the integrated encoders from the two right motors.
    double getRightEnc();

    ///\return The average actual velocity of the two left motors.
    double getLeftVel();

    ///\return The average actual velocity of the two right motors.
    double getRightVel();

    /**
     * Moves the two left motors using voltage instead "RPM".
     *
     * The normal moveVelocity function controls the motor speeds with a PID 
     * and can potentially cause unwanted behaviour. This avoids those issues.
     * 
     * \param RPM supposedly the target RPM of the motor. 
     * It's really more of a "percent power". The motors will spin at RPM/200 percent power.
     * 
     * \return True if successful (current implementation always is).
     */
    bool leftMoveRPM(double);

    /**
     * Moves the two right motors using voltage instead "RPM".
     *
     * The normal moveVelocity function controls the motor speeds with a PID 
     * and can potentially cause unwanted behaviour. This avoids those issues.
     * 
     * \param RPM supposedly the target RPM of the motor. 
     * It's really more of a "percent power". The motors will spin at RPM/200 percent power.
     * 
     * \return True if successful (current implementation always is).
     */
    bool rightMoveRPM(double);

    /**
     * Moves the all four drive motors using voltage instead "RPM".
     *
     * The normal moveVelocity function controls the motor speeds with a PID 
     * and can potentially cause unwanted behaviour. This avoids those issues.
     * 
     * \param RPM supposedly the target RPM of the motor. 
     * It's really more of a "percent power". The motors will spin at RPM/200 percent power.
     * 
     * \return True if successful (current implementation always is).
     */
    void moveRPM(double);

    /**
     * Detect any stalling drive motors using their actual velocity and torque.
     *  
     * \return True if any drive motors are stalling, false otherwise.
     */
    bool getStalling();

    /// Resets the integrated encoders of all the drive motors.
    void resetEncoders();

    /// Sets all drive motors' brake mode to "brake", which makes them resist movement when not moving.
    void brake();

    /// Sets all drive motors' brake mode to "coast", which allows them to move freely when not moving.
    void coast();
    
    /// Sets all drive motors' brake mode to "hold", which actively keeps them in-place.
    /// This will overheat motors if overused.
    void hold();
};
#endif /* _DRIVE_HPP_INCLUDED */
