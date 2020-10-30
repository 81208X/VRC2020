#include "auto.hpp"

#include <algorithm>
#include <vector>
#include <cstdlib>
#include "systemmanager.hpp"
#include "util/util.hpp"
#include "subsystem.hpp"
#include "odometry.hpp"

util::Pos2d targetPoint;
double speed;
double turningSpeed;
double tolerance;
bool isStopAtEnd;
bool absLimit;
double targetAngle;
double angleTolerance;
double strafeDistance;
int timeout;

bool AutoDrive::isSettled() {
	return flag == IDLE;
}

void AutoDrive::stop() {
	if(flag) {
		localStorage.log("deleted a task");
		pros::c::task_delete(autoTask);
		flag = IDLE;
	}
}

AutoDrive::AutoDrive() {
	resetSettings();
}

// The main logic for automatic moving, handles both driving to point and turn to angle
void autoTaskFn(void *param) {
	AutoDrive *auton = (AutoDrive*) param;
	double lastErrD = 0, lastErrA = 0, lastPower = 0;
	double power, turn, strafe, errD, errA;

	#pragma GCC diagnostic push
	// dT is unused, but kept in case we end up using Δtime in auto for something
	#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
	uint32_t nowTime = pros::millis(), lastTime = nowTime, dT = 0;
	#pragma GCC diagnostic pop
	int stalling = 0, steadyState = 0;

	// Creates the positional iterator pid controllers using pre-tuned values, specific to each robot.
	auto powerController = okapi::IterativeControllerFactory::posPID(FORWARD_P, FORWARD_I, FORWARD_D);
	auto strafeController = okapi::IterativeControllerFactory::posPID(STRAFE_P, STRAFE_I, STRAFE_D);
	auto turningController = okapi::IterativeControllerFactory::posPID(TURNING_P, TURNING_I, TURNING_D);
	// Sets the target of the PID controllers to 0.
	// Error values will be fed in as current value. The PID controllers will try to minimize that.
	powerController.setTarget(0);
	turningController.setTarget(0);
	strafeController.setTarget(0);

	// Creates the slewrate limiters, which limits the rate the speed of the robot changes, using pre-tuned values.
	// This prevents things like tipping and jumping from sudden change in wheel speed.
	util::SlewRateLimiter powerSlewRateLimiter(FORWARD_ACCEL, odometry.getChassisVel().y, FORWARD_DECEL);
	util::SlewRateLimiter strafeSlewRateLimiter(STRAFE_ACCEL,odometry.getChassisVel().x,STRAFE_DECEL);
	util::SlewRateLimiter turnSlewRateLimiter(TURNING_ACCEL, odometry.getChassisVel().angle, TURNING_DECEL);


	util::Pos2d closestPoint, closestPointSide;
	double angleToClose, angleToTarget, distanceToClose, distanceToTarget, distanceToCloseSide, angleToCloseSide;
	util::ChassisPos pos, sidewayPos;
	
	uint32_t startingTime = pros::millis();

	// character buffer for printing things to the log
	char logBuf[80];
	if(auton->flag == AutoDrive::AutoFlag::DRIVING_TO_POINT)
		sprintf(logBuf, "Starting Auton with target point %.2f %.2f", targetPoint.x, targetPoint.y);
	else if(auton->flag == AutoDrive::AutoFlag::TURNING)
		sprintf(logBuf, "Starting Auton with target angle %.2f", targetAngle);
	localStorage.log(logBuf);

	// "do..while loop" so we can run the logic first to fill up the variables.
	// This way we don't have to write any additional initialization code.
	do {
		// Handles timeout
		if(startingTime + timeout < pros::millis()){
			sprintf(logBuf, "Auto timeout");
			localStorage.log(logBuf);
			break;
		}

		// "wakes" any settled controller as we are still in the loop and the robot is still moving
		if(turningController.isSettled())
			turningController.setTarget(0);
		if(powerController.isSettled())
			powerController.setTarget(0);
		if(strafeController.isSettled())
			strafeController.setTarget(0);

		// Updates Time (technically unused)
		nowTime = pros::millis();
		dT = nowTime - lastTime;
		lastTime = nowTime;

		// Updates the current position of the robot
		pos = odometry.getPos();

		// The robot's position but sideways. Used to calculate strafing related values
		sidewayPos = {pos.x, pos.y, util::wrapAngle(pos.angle + (M_PI/2.0))};

		// Updates related points and values
		closestPoint = pos.getClosestPointAsHeading(targetPoint);
		closestPointSide = sidewayPos.getClosestPointAsHeading(targetPoint);
		angleToClose = pos.getAngleToAsHeading(closestPoint);
		angleToTarget = pos.getAngleToAsHeading(targetPoint);
		angleToCloseSide = sidewayPos.getAngleToAsHeading(closestPointSide);
		distanceToClose = pos.distance(closestPoint);
		distanceToTarget = pos.distance(targetPoint);
		distanceToCloseSide = sidewayPos.distance(closestPointSide);

		// Prints debugging information to the V5 brain
		pros::lcd::print(5, "Closest: %.2f %.2f", closestPoint.x, closestPoint.y);

		// Flips the direction of error based on the robot's facing.
		// Allows the robot to approach the target by backing up, if the target is behind the robot
		if(abs(angleToClose) >= M_PI/2.0)
			distanceToClose *= -1;
		if(abs(angleToCloseSide) >= M_PI/2.0)
			distanceToCloseSide *= -1;

		if(abs(distanceToTarget) < strafeDistance) {
			errA = 0;
			errD = distanceToClose;
		} else {
			errA = angleToTarget;
			errD = distanceToTarget;
		}

		// Stepping the PID controllers
		// Steps differently based on if we are driving to point or turning to a heading
		if(auton->flag == 2) {
			errA = util::wrapAngle(targetAngle - pos.angle);
			turningController.step(-errA);
			powerController.step(0);
			strafeController.step(0);
		}
		else {
			errA = util::wrapAngle90(errA);
			turningController.step(-errA);
			powerController.step(-distanceToClose);
			strafeController.step(-distanceToCloseSide);
		}

		// Getting the PID controller outputs
		power = powerController.getOutput() * speed;
		turn = turningController.getOutput() * turningSpeed * speed;
		strafe = strafeController.getOutput() * speed;
		
		// Limits the rate of change for the three values
		power = powerSlewRateLimiter.calculate(power);
		turn = turnSlewRateLimiter.calculate(turn);
		strafe = strafeSlewRateLimiter.calculate(strafe);
		
		// Detecting stalling
		if (drive.getStalling()) {
			stalling++;
			sprintf(logBuf, "Drive Stalling: %d", stalling);
			if(stalling%5 == 0)
				localStorage.log(logBuf);
		}
		else {
			if (stalling > 0)
				stalling -= 5;
			if(stalling < 0)
				stalling = 0;
		}
		// Detecting steadystate
		if (abs(errD - lastErrD) < (0.1 / 100.0) && abs(errA - lastErrA) < (d2r(10) / 100.0) && abs(power - lastPower) < (0.1 / 100.0)) {
			steadyState++;
			sprintf(logBuf, "Auto Steady State: %d", steadyState);
			if(steadyState%5 == 0)
				localStorage.log(logBuf);
		}
		else {
			if (steadyState > 0)
				steadyState -= 5;
			if(steadyState < 0)
					steadyState = 0;
		}

		// Breaking if the robot is stuck or not moving
		if (stalling+steadyState > 15) {
			sprintf(logBuf, "Breaking due to steady or stalling");
			localStorage.log(logBuf);
			break;
		}

		// applies the calculated velocity values to the drive base
		drive.setChassisSpeedIK({strafe * MAX_SPEED_IN_S * M_SQRT2, power * MAX_SPEED_IN_S * M_SQRT2, turn * MAX_CHASSIS_RPS}, absLimit);

		// Updates "last" values used to calculate changes
		lastErrA = errA;
		lastErrD = errD;
		if(robotConfigs.debugging)
			pros::lcd::print(AUTO_LCD_LINE, "power:%.1f, turn:%.1f, strafe:%.1f", power, turn, strafe);

		pros::delay(10);
		
		// Runs the loop until all controllers have settled and we have reached the target
	} while(
		(!powerController.isSettled()) || 
		(!strafeController.isSettled()) || 
		(!turningController.isSettled()) || 
		(auton->flag == 1 && errD > tolerance) || 
		(auton->flag == 2 && abs(errA) > angleTolerance));

	if (isStopAtEnd) {
		drive.leftMoveRPM(0);
		drive.rightMoveRPM(0);
	}
	auton->flag = AutoDrive::AutoFlag::IDLE;
}

bool AutoDrive::driveToPointAsync(const util::Pos2d input) {
	// Stops any current automatic movements, if any. 
	stop();
	pros::delay(20);
	targetPoint = input;
	flag = AutoFlag::DRIVING_TO_POINT;
	autoTask = pros::c::task_create(autoTaskFn, this, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "AutoDrive Task");

	return true; // This function is async and will always succeed (does not incicate status of autotask).
}

bool AutoDrive::turnToAngleAsync(double input) {
	// Stops any current automatic movements, if any. 
	stop();
	pros::delay(20);
	targetAngle = input;
	flag = AutoFlag::TURNING;
	autoTask = pros::c::task_create(autoTaskFn, this, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "AutoDrive Task");
	return true; // This function is async and will always succeed (does not incicate status of autotask).
}

// --------- Functions for setting drive settings --------- //
AutoDrive& AutoDrive::withTolerance(const double input) {
	tolerance = input;
	return *this;
}
AutoDrive& AutoDrive::withSpeed(double input) {
	speed = input;
	return *this;
}
AutoDrive& AutoDrive::withTurnSpeed(double input) {
	turningSpeed = input;
	return *this;
}
AutoDrive& AutoDrive::stopAtEnd(bool input) {
	isStopAtEnd = input;
	return *this;
}
AutoDrive& AutoDrive::withMaxMotorSpeed(double speed) {
	absLimit = speed;
	return *this;
}
AutoDrive& AutoDrive::withAngleTolerance(double input) {
	angleTolerance = input;
	return *this;
}
AutoDrive& AutoDrive::withStrafeDistance(double input) {
	strafeDistance = input;
	return *this;
}
AutoDrive& AutoDrive::withTimeout(int input) {
	timeout = input;
	return *this;
}
AutoDrive& AutoDrive::resetSettings() {
	speed = speedDefault;
	turningSpeed = turningSpeedDefault;
	tolerance = toleranceDefault;
	isStopAtEnd = isStopAtEndDefault;
	absLimit = absLimitDefault;
	angleTolerance = angleToleranceDefault;
	strafeDistance = strafeDistanceDefault;
	timeout = timeoutDefault;
	return *this;
}
