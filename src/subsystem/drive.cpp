#include "drive.hpp"

#include <cmath>
#include "okapi/api.hpp"

#include "util/util.hpp"
#include "io.hpp"
#include "profiles.hpp"

#define lfm leftFwdMtr
#define rfm rightFwdMtr
#define lrm leftRearMtr
#define rrm rightRearMtr

DriveSubsystem::DriveSubsystem() : IK({0, 0}) {
    initDriveCurveLookup();
    coast();
}

void DriveSubsystem::initDriveCurveLookup() {
    //The "curve" function is made in desmos https://www.desmos.com/calculator/ntm9m2popj
    //The current version of the function on desmos has a domain and range of -1 to +1, but the math is quite similar
    int input;
    double a, b;
    for (uint8_t x = 0; x <= 254; x++) {
        input = x - 127;
        a = exp(MANUAL_DRIVE_CURVATURE * -0.1);
        b = (a + exp(0.1 * (fabs(input) - 127)) * (1 - a));
        driveCurveLookUp[x] = (int)round(b * input);
    }
}

int DriveSubsystem::lookupDriveCurve(int8_t input) {
    return driveCurveLookUp[input + 127];
}

void DriveSubsystem::handleDriver() {
    // Gets controller input, then remaps it to a range of -1 to +1
    double power = okapi::remapRange(lookupDriveCurve(controllerMaster.getAxis(DRIVE_POWER)), -128, 128, -1, 1);
    double strafe = okapi::remapRange(lookupDriveCurve(controllerMaster.getAxis(DRIVE_STRAFE)), -128, 128, -1, 1);
    double turn = okapi::remapRange(lookupDriveCurve(controllerMaster.getAxis(DRIVE_TURN)), -128, 128, -1, 1);

    // Using inverse kinematics to generate wheelspeeds
    // There are definitely simpler ways of doing this.
    // We used this to debug our inverse kinematics code, and it works, so we didn't bother changing it.

    // Wheelspeed: the speed at which each wheel should be turning at.
    util::WheelSpeed ws = IK.toWheelSpeed({strafe * MAX_SPEED_IN_S * M_SQRT2, power * MAX_SPEED_IN_S * M_SQRT2, turn * MAX_CHASSIS_RPS}, {0,0});
    // Normalizes the wheel speeds to make sure the target speed is reachable by our motors
    ws.normalize(MAX_SPEED_IN_S);
    // Sets the calculated wheelspeed to the drivebase.
    setWheelSpeed(ws, 200.0 / MAX_SPEED_IN_S);
}

void DriveSubsystem::driveSimple(util::ChassisSpeed cs) {
    util::WheelSpeed wheelSpeed = {cs.x + cs.y + cs.angle, -cs.x + cs.y + cs.angle, -cs.x + cs.y - cs.angle, cs.x + cs.y - cs.angle};
    wheelSpeed.normalize(1.0);
    setWheelSpeed(wheelSpeed, MAXRPM);
}

void DriveSubsystem::setChassisSpeedIK(util::ChassisSpeed cs, double maxMotorSpeedMultiplier) {
    maxMotorSpeedMultiplier = std::clamp(maxMotorSpeedMultiplier, 0.0, 1.0);
    setWheelSpeed(IK.toWheelSpeed(cs, {0,0}).normalize(MAX_SPEED_IN_S * maxMotorSpeedMultiplier), 200 / MAX_SPEED_IN_S);
}


void DriveSubsystem::setWheelSpeed(util::WheelSpeed wheelSpeed, double multiplier) {
    lfm.moveVelocity(wheelSpeed.lf * multiplier);
    lrm.moveVelocity(wheelSpeed.lr * multiplier);
    rfm.moveVelocity(wheelSpeed.rf * multiplier);
    rrm.moveVelocity(wheelSpeed.rr * multiplier);
}

void DriveSubsystem::coast() {
    driveMtrGrp.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
}

void DriveSubsystem::brake() {
    driveMtrGrp.setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
}

void DriveSubsystem::hold() {
    driveMtrGrp.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
}

double DriveSubsystem::getLeftEnc() {
    return util::avgDouble(lfm.getPosition(), lrm.getPosition()) / 900.0 * 360.0;
}
double DriveSubsystem::getRightEnc() {
    return util::avgDouble(rfm.getPosition(), rrm.getPosition()) / 900.0 * 360.0;
}
double DriveSubsystem::getLeftVel() {
    return util::avgDouble(lfm.getActualVelocity(), lrm.getActualVelocity());
}
double DriveSubsystem::getRightVel() {
    return util::avgDouble(rfm.getActualVelocity(), rrm.getActualVelocity());
}
bool DriveSubsystem::leftMoveRPM(double speed) {
    leftMtrGrp.moveVoltage(speed / 200.0 * 12000.0);
    return true;
}

void DriveSubsystem::moveRPM(double speed) {
    driveMtrGrp.moveVoltage(speed / 200.0 * 12000.0);
}

bool DriveSubsystem::rightMoveRPM(double speed) {
    rightMtrGrp.moveVoltage(speed / 200.0 * 12000.0);
    return true;
}

void DriveSubsystem::resetEncoders() {
    driveMtrGrp.tarePosition();
}

bool DriveSubsystem::getStalling() {
    return  (lfm.getTorque() > DRIVE_STALL_TORQUE && abs(lrm.getActualVelocity()) < DRIVE_STALL_VELOCITY) ||
            (lrm.getTorque() > DRIVE_STALL_TORQUE && abs(lrm.getActualVelocity()) < DRIVE_STALL_VELOCITY) ||
            (rfm.getTorque() > DRIVE_STALL_TORQUE && abs(rfm.getActualVelocity()) < DRIVE_STALL_VELOCITY) ||
            (rrm.getTorque() > DRIVE_STALL_TORQUE && abs(rrm.getActualVelocity()) < DRIVE_STALL_VELOCITY);
}
