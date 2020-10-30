#ifndef _INTAKE_HPP_INCLUDED
#define _INTAKE_HPP_INCLUDED

#include "api.h"

class IntakeSubsystem {
private:
    int speed = 12000; // Speed is in milivolts
    
public:
    /**
     * -1 = moving  
     * 0 = moving intake manually
     * 1 = stopping intake after the user stops giving manual inputs
     */
    int flag = 0;

    IntakeSubsystem();

    /**
     * Wrapper function for moveRelative() from the MotorGroup class. Also sets the appropriate flag.
     * 
     * \param iposition The relative position to move to in the motor’s encoder units.
     * 
     * \param ivelocity The maximum allowable velocity for the movement in RPM, assuming the motors move at 200RPM.
     */
    void moveRelative(double iposition, int32_t ivelocity);

    ///Reacts to controller input and updates motor speeds accordingly. Should be called on every loop in opcontrol.
    void handleDriver();

    /// Sets all drive motors' brake mode to "brake", which makes them resist movement when not moving.
    void brake();

    /// Sets all drive motors' brake mode to "coast", which allows them to move freely when not moving.
    void coast();
    
    /// Sets all drive motors' brake mode to "hold", which actively keeps them in-place. 
    void hold();
    
    /**
     * Wrapper function for moveVoltage() from the MotorGroup class.
     * Sets the voltage for all intake motors.
     * 
     * \param voltage The new voltage value from -12000 to 12000.
     */
    void moveVoltage(int voltage);

    /**
     * Wrapper function for moveVelocity() from the MotorGroup class.
     * Sets the velocity for all intake motors.
     * 
     * \param velocity The new motor velocity from -200 to +200. The motor is assumed to be running at 200rpm.
     */
    void moveVelocity(int velocity);
};
#endif /* _INTAKE_HPP_INCLUDED */
