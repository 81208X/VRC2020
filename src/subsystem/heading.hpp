#ifndef _HEADING_HPP_INCLUDED
#define _HEADING_HPP_INCLUDED

#include "api.h"
#include "okapi/api.hpp"

// Abstracts IMU and gyro into a single class, allowing either to be polled in the same unit.
class HeadingSensor {
private:
    bool useInertialSensor; // This gates code paths based on IMU/gyro, should maybe change to a compile-time macro(?)
    okapi::ADIGyro* gyro;
    pros::Imu* inertialSensor;
    
public:
    HeadingSensor(okapi::ADIGyro*);
    HeadingSensor(pros::Imu*);

    /**
	 * Gets the heading in degrees.
     *
     * \return Value from the sensor in degrees. This value is wrapped to the range
     * of -180 to 180.
	 */
    double getDegrees();

    /// Prints the value of the sensor to the screen. Meant to be called on a loop.
    void debug();

    /**
     * Resets the heading sensor's zero position and recalibrates.
     * Must be stationary before calling or results can be unpredictable.
     */
    void reset();

    /**
     * Initializes the physical sensor.
     * Blocks for ~3 seconds if using IMU, so is seperate from the constructor
     */
    void initialize();
};
#endif /* _HEADING_HPP_INCLUDED */
