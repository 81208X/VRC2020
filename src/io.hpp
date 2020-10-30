// Direct access to hardware in the global namespace
// Forward declared here and initialized in main for the most part

// Please abstract hardware access into subsystems so we can add 
// things like mutexes to prevent concurency issues

#ifndef _IO_HPP_INCLUDED
#define _IO_HPP_INCLUDED

#include "api.h"
#include "okapi/api.hpp"

#include "io/sdcard.hpp"
#include "io/controller.hpp"

// default config access
extern LocalStorage::RobotConfigs robotConfigs;

extern LocalStorage localStorage;
extern Controller controllerMaster;

/* -------------- Hardware -------------- */
extern okapi::Motor leftFwdMtr, rightFwdMtr, leftRearMtr, rightRearMtr;
extern okapi::MotorGroup leftMtrGrp, rightMtrGrp, driveMtrGrp;

extern okapi::Motor leftIntakeMtr, rightIntakeMtr;
extern okapi::MotorGroup intakeMtrGrp;

extern okapi::ADIEncoder backEnc;

// Functions that can be disabled by removing their entry in
// profiles.hpp for testing. Do not disable without checking
// for use in the autoRoutine (for auton) testing and codependencies.
#ifdef LEFT_ENCODER
extern okapi::ADIEncoder leftEnc;
#endif
#ifdef RIGHT_ENCODER
extern okapi::ADIEncoder rightEnc;
#endif

#ifdef UPPER_ROLLER_MOTOR
extern okapi::Motor upperRoller;
#endif
#ifdef LOWER_ROLLER_MOTOR
extern okapi::Motor lowerRoller;
#endif

extern okapi::MotorGroup rollerMtrGrp; // Contains no motors if left/right disabled

#ifdef INDEXER_FRONT
extern pros::ADIAnalogIn frontIndexer;
#endif
#ifdef INDEXER_BACK
extern pros::ADIAnalogIn backIndexer;
#endif

extern pros::Imu inertialSensor;
#endif /* _IO_HPP_INCLUDED */
