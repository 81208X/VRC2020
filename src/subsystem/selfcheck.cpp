#include "selfcheck.hpp"

#include <climits>
#include <cstdio>
#include "profiles.hpp"
#include "io.hpp"
#include "subsystem.hpp"

/**
 * When the temperature is higher than 100C, which is considered to be impossible, it will be treated as 0C.
 * The only situation where the temperature will be higher than 100C is if the motor is disconnected.
 * In that case, the motor temperature will be an insanely large value.
 * 
 * This is to prevent disconnected motors from clogging up the "highest temperature motor" display.
 */
#define MOTOR_TEMP pros::c::motor_get_temperature(port)>100?0:pros::c::motor_get_temperature(port)
#define MOTOR_FAULT pros::c::motor_get_faults(port)

SelfCheck::SelfCheck(std::vector<int> portsUsed) {
	ports = portsUsed;
}

void selfCheckTaskFn(void *param) {
	pros::delay(500);
	char strBuf[20];
	while(true) {
		bool errFlag = false;
		int maxTemp = 0, maxPort = 0;
		for(int port:selfCheck.ports) {
			double curTemp = MOTOR_TEMP; // Store temperature because motors dont like being polled too quick

			if(curTemp >= maxTemp) {
				maxTemp = curTemp;
				maxPort = port;
			}

			bool error = true;

			if(MOTOR_FAULT==INT_MAX)
				//motor's not connected!
				sprintf(strBuf, "Port %d DISCON  ", port);
			else if(curTemp >= MOTOR_OVERHEAT_TEMP)
				//motor overheating
				sprintf(strBuf, "Port %d: %.0F", port, curTemp);
			else
				error = false;
	
			/* If the motor is having an error, print to the controller immediately */
			if(error){
				controllerMaster.print(strBuf);
				//waits 2 seconds after printing a message because the controller doesn't like being printed too frequently
				uint32_t now = pros::millis();
				pros::Task::delay_until(&now, 2000);
			}
    }
		/** 
		 * Rumbles the controller if any error occurred.
		 * To conserve controller battery. this is only done once per check, instead of once per error.
		 */
		if(errFlag)
			controllerMaster.rumble("-"); // Rumble string doesn't seem too well documented but this works well enough
		else{
			// Prints the motor with the maximum temperature if there are no error.
			switch(maxPort) { // Strings are kept short because the controller LCD only does like 15 characters
				case LEFT_FORWARD_MOTOR:
				case RIGHT_FORWARD_MOTOR:
				case LEFT_REAR_MOTOR:
				case RIGHT_REAR_MOTOR:
					sprintf(strBuf, "DRIVE");
					break;
				case LEFT_INTAKE_MOTOR:
				case RIGHT_INTAKE_MOTOR:
					sprintf(strBuf, "INTAK");
					break;
				case UPPER_ROLLER_MOTOR:
				case LOWER_ROLLER_MOTOR:
					sprintf(strBuf, "ROLL ");
			}
			sprintf(strBuf+strlen(strBuf), ": %d on #%d  ", maxTemp, maxPort);
			controllerMaster.print(strBuf);
		}
		//Delay 2000 seconds because controller doesn't like being printed too frequently
		pros::delay(2000);
	}
}

void SelfCheck::startTask() {
	if(!taskRunning) {
		taskRunning = true;
		selfCheckTask = pros::c::task_create(selfCheckTaskFn, this, TASK_PRIORITY_MIN,
			TASK_STACK_DEPTH_DEFAULT, "Self Check Task");
	}
}

void SelfCheck::endTask() {
	if (taskRunning) {
		pros::c::task_delete(selfCheckTask);
		taskRunning = false;
	}
}
