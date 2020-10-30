#include "heading.hpp"

#include "profiles.hpp"
#include "util/util.hpp"

HeadingSensor::HeadingSensor(okapi::ADIGyro *adigyro) {
    useInertialSensor = false;
    gyro = adigyro;
    gyro->reset();
}

HeadingSensor::HeadingSensor(pros::Imu *inertialSen) {
    useInertialSensor = true;
    inertialSensor = inertialSen;
}

void HeadingSensor::initialize() {
    if(!useInertialSensor)
        return;
    pros::lcd::print(GYRO_LCD_LINE, "(gyro) CALIBRATING!");
    inertialSensor->reset();
    util::blocking([&]{ return !inertialSensor->is_calibrating(); });
    pros::lcd::print(GYRO_LCD_LINE, "(gyro) done calibrating");
    pros::delay(750); // leo said to delay more so sure
    pros::lcd::clear_line(GYRO_LCD_LINE);
}

void HeadingSensor::reset() {
    gyro->reset();
}

void HeadingSensor::debug() {
    pros::lcd::print(GYRO_LCD_LINE, "(gyro) val: %.1f", getDegrees());
}

double HeadingSensor::getDegrees() {
    if(useInertialSensor)
        return r2d(util::wrapAngle(d2r(inertialSensor->get_heading())));
    else
        return r2d(util::wrapAngle(d2r(gyro->getRemapped(360, -360))));
}
