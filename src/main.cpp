#include "main.h"

#include <cmath>
#include <vector>
#include <array>

#include "profiles.hpp"
#include "io.hpp"
#include "subsystem.hpp"
#include "systemmanager.hpp"
#include "menu.hpp"
#include "autoRoutine.hpp"

using namespace okapi::literals;

// Default config
LocalStorage::RobotConfigs robotConfigs = {
	// Driver skills
#ifdef DRIVERSKILLS
	true,
#else
	false,
#endif
	0,     			// Auton mode
	LocalStorage::AutonMode::RED_BOTTOM, // Side
	false,			// Debug mode
	true,			// Logging enable
	STARTING_Y_IN	// Starting Y value in inches
};

//Base drive
okapi::Motor leftFwdMtr(LEFT_FORWARD_MOTOR, LEFT_FORWARD_MOTOR_REVERSED, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::degrees);
okapi::Motor rightFwdMtr(RIGHT_FORWARD_MOTOR, RIGHT_FORWARD_MOTOR_REVERSED, okapi::AbstractMotor::gearset::green,okapi::AbstractMotor::encoderUnits::degrees);
okapi::Motor leftRearMtr(LEFT_REAR_MOTOR, LEFT_REAR_MOTOR_REVERSED, okapi::AbstractMotor::gearset::green,okapi::AbstractMotor::encoderUnits::degrees);
okapi::Motor rightRearMtr(RIGHT_REAR_MOTOR, RIGHT_REAR_MOTOR_REVERSED, okapi::AbstractMotor::gearset::green,okapi::AbstractMotor::encoderUnits::degrees);
okapi::MotorGroup leftMtrGrp = { leftFwdMtr, leftRearMtr };
okapi::MotorGroup rightMtrGrp = { rightFwdMtr, rightRearMtr };
okapi::MotorGroup driveMtrGrp = { leftFwdMtr, leftRearMtr, rightFwdMtr, rightRearMtr };


okapi::Motor leftIntakeMtr(LEFT_INTAKE_MOTOR, LEFT_INTAKE_MOTOR_REVERSED, okapi::AbstractMotor::gearset::green,okapi::AbstractMotor::encoderUnits::degrees);
okapi::Motor rightIntakeMtr(RIGHT_INTAKE_MOTOR, RIGHT_INTAKE_MOTOR_REVERSED, okapi::AbstractMotor::gearset::green,okapi::AbstractMotor::encoderUnits::degrees);
okapi::MotorGroup intakeMtrGrp = {leftIntakeMtr, rightIntakeMtr};

#ifdef UPPER_ROLLER_MOTOR
okapi::Motor upperRoller(UPPER_ROLLER_MOTOR, UPPER_ROLLER_MOTOR_REVERSED, okapi::AbstractMotor::gearset::blue, okapi::AbstractMotor::encoderUnits::degrees);
#endif
#ifdef LOWER_ROLLER_MOTOR
okapi::Motor lowerRoller(LOWER_ROLLER_MOTOR, LOWER_ROLLER_MOTOR_REVERSED, okapi::AbstractMotor::gearset::blue, okapi::AbstractMotor::encoderUnits::degrees);
#endif

// This is a mess but I am really sticking to modularity done through macros
// With this we can remove a roller without breaking the roller group
okapi::MotorGroup rollerMtrGrp = {
#ifdef UPPER_ROLLER_MOTOR
    upperRoller
#endif
#if defined UPPER_ROLLER_MOTOR && defined LOWER_ROLLER_MOTOR
    ,
#endif
#ifdef LOWER_ROLLER_MOTOR
    lowerRoller
#endif
};

okapi::ADIEncoder backEnc(BACK_ENCODER_TOP, BACK_ENCODER_BOTTOM, BACK_ENCODER_REVERSED);
okapi::ADIEncoder leftEnc(LEFT_ENCODER_TOP, LEFT_ENCODER_BOTTOM, LEFT_ENCODER_REVERSED);
okapi::ADIEncoder rightEnc(RIGHT_ENCODER_TOP, RIGHT_ENCODER_BOTTOM, RIGHT_ENCODER_REVERSED);

LocalStorage localStorage;
Controller controllerMaster(pros::E_CONTROLLER_MASTER);

//Subsystems
DriveSubsystem drive;
IntakeSubsystem intake;
SelfCheck selfCheck(std::vector<int> (SELFCHECK_PORTS));

pros::Imu inertialSensor(INERTIAL_SENSOR);
HeadingSensor gyroSystem(&inertialSensor); // Also supports being passed a gyro instead of IMU

#ifdef VISION_SENSOR_LOWER
VisionSensor visionSensorLower(VISION_SENSOR_LOWER, VISION_LOWER_RED_SIG, VISION_LOWER_BLUE_SIG, VISION_LOWER_SIG_MIN_WIDTH, VISION_LOWER_SIG_MIN_HEIGHT);
#endif

#ifdef VISION_SENSOR_UPPER
VisionSensor visionSensorUpper(VISION_SENSOR_UPPER, VISION_UPPER_RED_SIG, VISION_UPPER_BLUE_SIG, VISION_UPPER_SIG_MIN_WIDTH, VISION_UPPER_SIG_MIN_HEIGHT);
#endif

#ifdef INDEXER_FRONT
pros::ADIAnalogIn frontIndexer(INDEXER_FRONT);
#endif
#ifdef INDEXER_BACK
pros::ADIAnalogIn backIndexer(INDEXER_BACK);
#endif

// Subsystem managers
Odometry odometry;
AutoDrive auton;
Menu menu;
Indexer indexer;

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize();
    localStorage.readConfigs();
    gyroSystem.initialize();

    odometry.resetForAuton();

    menu.controllerNavigation = true;
    menu.startTask();
    selfCheck.startTask();

    rollerMtrGrp.setBrakeMode(okapi::AbstractMotor::brakeMode::hold); // Consider brake so it heats up less?
    intake.brake();
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
    // Coasts all the motors when robot is disabled for easier removal from field and avoid motor stalling
    drive.coast();
    intake.coast();
    rollerMtrGrp.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);

    menu.startTask();
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {} // This is pretty useless because we mostly debug without
                                 // the competition switch and it would go untested.

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
    util::runAsync([&]{
        localStorage.log("--- START OF AUTON ---");
        drive.brake();
        if(!robotConfigs.debugging) {
            menu.endTask(); // Also stops the block on opcontrol
            pros::lcd::shutdown(); // Shutdown LLEMU used for debug
            menu.initGraphics(); // Init screensaver thing
        }
    });

    odometry.resetForAuton();
    odometry.startTask();
    localStorage.log("Resetted odometry");
    pros::delay(15);

    autoRoutine::skillsAuton(); // Auton selection logic has been removed for this branch
                                // This code will only be used for skills anyways
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
    drive.brake();
    intake.moveVoltage(0);
    drive.moveRPM(0);

    if(robotConfigs.debugging && !odometry.taskRunning)
        odometry.startTask();

    if(robotConfigs.driverSkills) // disable controller menu navigation if in driver skills
        menu.controllerNavigation = false;

    // delay 50ms on starting opcontrol or odometry can sometimes have problems
    // opcontrol loop poll frequency is every 20ms, 50Hz
    for(pros::delay(50); true; pros::delay(20)) {
        // Print stalls on any drive motor to log for later analysis
        if(drive.getStalling() && robotConfigs.debugging)
            localStorage.log("Stalling!" "\a"); // Beep if stalling

        // Disable opcontrol-related stuff in test builds
        #ifdef TEST_BUILD
        continue;
        #endif

        // disable driver control while in menu mode
        if (menu.controllerNavigation)
            continue;

        // Debugging indexer/sorter buttons
        #ifndef FORCE_COMPETITION // Disable for comp
        if (robotConfigs.debugging) {
            if(controllerMaster.getBtnNew(DEBUG_SORTER_Y))
                indexer.getUpperBall();
            else if(controllerMaster.getBtnNew(DEBUG_SORTER_B))
                indexer.getLowerBall();
            else if(controllerMaster.getBtnNew(DEBUG_SORTER_DOWN))
                indexer.score();
        }
        #endif

        /**
         * Runs a custom automatic driving function if the robot is in debug mode and debug button is pressed.
         * 
         * Stops any automatic driving immediately if the debug button is released.
         * This helps us in preventing the robot from hurting it-self from bad code.
         */
        if(robotConfigs.debugging) {
            switch (auton.flag) {
                case AutoDrive::AutoFlag::IDLE:
                    if(controllerMaster.getBtnNew((DEBUG_STRAIGHT))) {
                        drive.brake();
                        auton.driveToPointAsync({0,24});
                    }
                    else
                        drive.handleDriver();
                    break;
                default:
                    if (!controllerMaster.getBtnRaw(DEBUG_STRAIGHT))
                        auton.stop();
                    break;
            }
        }
        else // Not debugging
            drive.handleDriver();
    }
}
