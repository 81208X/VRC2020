#include "intake.hpp"

#include "okapi/api.hpp"
#include "profiles.hpp"
#include "io.hpp"

IntakeSubsystem::IntakeSubsystem() {
    coast();
}

void IntakeSubsystem::handleDriver() {
    if (controllerMaster.getBtnRaw(INTAKE_OUT)) {
        flag = 0;
        intakeMtrGrp.moveVoltage(speed);
    }
    else if (controllerMaster.getBtnRaw(INTAKE_IN)) {
        flag = 0;
        intakeMtrGrp.moveVoltage(0 - speed);
    }
    // Hold intake
    else if (flag == 0) {
        flag = -1;
        //Sets the voltake to 0 first to immidietly stop moving
        //From our testing, the velocity PID seems like it might have some latency.
        intakeMtrGrp.moveVoltage(0);
        intakeMtrGrp.moveVelocity(0);
    }
    // Brake intake
    else if (flag == -1) {
        intakeMtrGrp.moveVelocity(0);
    }
}

void IntakeSubsystem::coast() {
    intakeMtrGrp.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
}

void IntakeSubsystem::brake() {
    intakeMtrGrp.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
}

void IntakeSubsystem::hold() {
    intakeMtrGrp.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
}

void IntakeSubsystem::moveRelative(double iposition, std::int32_t ivelocity) {
    flag = 1;
    intakeMtrGrp.moveRelative(iposition, ivelocity);
}

void IntakeSubsystem::moveVoltage(int voltage) {
    intakeMtrGrp.moveVoltage(voltage);
}

void IntakeSubsystem::moveVelocity(int velocity) {
    intakeMtrGrp.moveVelocity(velocity);
}
