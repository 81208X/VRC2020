#include "odometry.hpp"

#include <cmath>
#include "systemmanager.hpp"
#include "util/util.hpp"

/*
x: ⬅️ negative, ➡️ positive
y: ⬇️ negative, ⬆️ positive
a: ↩️ positive, ↪️ negative
*/
Odometry::Odometry() {
}

void Odometry::useGyroRotation(){
    if(gyroRotation) return;
    bool taskWasRunning = taskRunning;
    if(taskWasRunning)
        endTask();
    pros::delay(20);
    zeroPosA =  d2r(gyroSystem.getDegrees()) - data.angle;
    gyroRotation = true;
    if(taskWasRunning)
        startTask();
}

void Odometry::useEncoderRotation(){
    if(!gyroRotation)return;
    bool taskWasRunning = taskRunning;
    if(taskWasRunning)
        endTask();
    pros::delay(20);
    zeroPosA = ((leftEnc.get() - rightEnc.get()) * SIDE_ENC_TO_IN / ENC_BASE_WIDTH_IN) - data.angle;
    gyroRotation = false;
    if(taskWasRunning)
        startTask();
}

void Odometry::resetForAuton() {
    switch (robotConfigs.auton) {
        // Auton selection logic has been removed for this branch
        // This code will only be used for skills anyways
        case 0:
            reset({0, 0, 0});
            break;
        default:
            reset({-72 + 11.25/2 + CHASSIS_WIDTH / 2, STARTING_Y_IN, 0});
            break;
    }
}

void Odometry::reset(util::ChassisPos originPoint, bool hardware) {
    bool taskWasRunning = taskRunning;
    if (taskWasRunning)
        endTask();
    pros::delay(20);
    data.x = originPoint.x;
    data.y = originPoint.y;
    data.angle = d2r(originPoint.angle);
    zeroPosA = d2r(gyroSystem.getDegrees() - originPoint.angle);
    if (hardware) {
        drive.resetEncoders();
        backEnc.reset();
        leftEnc.reset();
        rightEnc.reset();
    }
    pros::delay(20);
    if (taskWasRunning)
        startTask();
}

void odometryTaskFn(void *param) {
    //Initialize the "last" encoder values
    double lastL = leftEnc.get();
    double lastR = rightEnc.get();
    double lastB = backEnc.get();

    //Initialize heading related values
    double lastA, lastGyro, newGyro;
    int gyroCounter = 0;
    if(odometry.gyroRotation) {
        lastA = lastGyro = d2r(gyroSystem.getDegrees()) - odometry.zeroPosA;
    } else {
        lastA = odometry.getPos().angle * (robotConfigs.autonSide & LocalStorage::AutonMode::RED ? 1.0 : -1.0);
    }

    double curL, curR, curB, dL, dR, dB, newA, dA, dX, dY, mA, chassisWidth, r, r2, i, sinI, cosMA, sinMA;

    //Getting the current position from the odometry class, as the task function is not a member of the class
	double x = odometry.getPos().x, y = odometry.getPos().y, angle = lastA;

    uint32_t time;
    pros::delay(20);
    while (true) {
        //Storing the encoder values in a local variable
        curL = leftEnc.get();
        curR = rightEnc.get();
        curB = backEnc.get();

        //Calculating the amount each tracking wheel as moved, in inches
        dL = (curL - lastL) * SIDE_ENC_TO_IN; // amount left side moved
        dR = (curR - lastR) * SIDE_ENC_TO_IN; // amount right side moved
        dB = (curB - lastB) * BACK_ENC_TO_IN; // amount back tracking wheel moved

        // If moving unreasonably fast, just ignore the inputs.
        // This might happen at the start of the program, or when the encoders somehow gets reset
        if (fabs(dL) > 1 || fabs(dR) > 1 || fabs(dB) > 1) {
            lastL = curL;
            lastR = curR;
            lastB = curB;
            pros::delay(5);
            continue;
        }

        //Store the chassis width to a local variable.
        //We might experiment with changing the chassiswidth while running in the future
        chassisWidth = ENC_BASE_WIDTH_IN;
        
        //Finding the new heading and difference in heading
        if(odometry.gyroRotation) {
            newGyro = gyroSystem.getDegrees();
            // The gyro only goes up to ±360 degrees, but we don't want it to jump from -360 to 360
            // We detect when this happens and makes the rotation continuous by adding the correct amount of rotation to it
            if (abs(lastGyro - newGyro) > 180 && abs(newGyro) > 100 &&
                abs(lastGyro) > 100 && signbit(newGyro) != signbit(lastGyro))
            {
                if (signbit(newGyro)) // counter clockwise
                    gyroCounter--;
                else // clockwise
                    gyroCounter++;
            }
            newA = d2r(gyroCounter * 360 + newGyro) - odometry.zeroPosA;
            dA = newA - lastA;
        } else {
            newA = (curL - curR) * SIDE_ENC_TO_IN / chassisWidth - odometry.zeroPosA;
            dA = newA - lastA;
        }

        odometry.setChassisVel({dB * (1000.0/odometry.delay) / (MAX_SPEED_IN_S / 2), util::avgDouble(dL,dR) * (1000.0/odometry.delay) / MAX_SPEED_IN_S, dA * (1000.0/odometry.delay) / MAX_CHASSIS_RPS});

        // http://thepilons.ca/wp-content/uploads/2018/10/Tracking.pdf
        if (dA == 0) {
            i = 0;
            dX = dB;
            dY = util::avgDouble(dL, dR);
        } else {
            r = dR / dA;
            r2 = dB / dA;
            i = dA / 2.0;
            sinI = sin(i);
            dX = 2 * sinI * (r2 + BACK_TO_CENTER_IN);
            dY = 2 * sinI * (r + chassisWidth / 2.0);
        }

        mA = lastA + i; // the angle between the last angle and the current angle,
                        // assumed to be the direction the robot moved in

        cosMA = std::cos(mA);
        sinMA = std::sin(mA);

        // Update the global position
        if (robotConfigs.autonSide & LocalStorage::AutonMode::RED) {
            angle = newA;
            x += (dY * sinMA) + (dX * cosMA);
        } else {
            angle = -newA;
            x -= (dY * sinMA) + (dX * cosMA);
        }
        y += (dY * cosMA) + (dX * -sinMA);

		odometry.setPos({x,y,angle});

#ifdef ODON_LCD_LINE_2
        if (robotConfigs.debugging)
            pros::lcd::print(ODON_LCD_LINE_2, "l: %.2f, r: %.2f, a: %.2f", curL, curR	, r2d(newA));
#endif

#ifdef ODON_LCD_LINE
        if (robotConfigs.debugging)
            pros::lcd::print(ODON_LCD_LINE, "x: %.2f, y: %.2f, a: %.2f", odometry.getPos().x,
                             odometry.getPos().y, r2d(odometry.getPos().angle));
#endif

        // char logBuf[80];
        // sprintf(logBuf, "Odometry at x: %.1f y: %.1f angle: %.1f degrees",
        //         odometry.getPos().x, odometry.getPos().y, r2d(odometry.getPos().angle));
        // localStorage.log(logBuf);

        //Updating the "Last" values
        lastA = newA;
        lastL = curL;
        lastR = curR;
        lastB = curB;

        time = pros::millis();
        pros::Task::delay_until(&time, odometry.delay); // TODO: try shorter time?
    }
}

void Odometry::startTask() {
    if (!taskRunning) {
        taskRunning = true;
        odoTask = pros::c::task_create(odometryTaskFn, this, TASK_PRIORITY_DEFAULT,
                                       TASK_STACK_DEPTH_DEFAULT, "odometry task");
    }
}

void Odometry::endTask() {
    if (taskRunning) {
        taskRunning = false;
        pros::c::task_delete(odoTask);
    }
}

util::ChassisPos Odometry::getPos(){
	return data;
}

void Odometry::setPos(util::ChassisPos pos){
	data = pos;
}

util::ChassisSpeed Odometry::getChassisVel(){
    return localVel;
}

void Odometry::setChassisVel(util::ChassisSpeed speed){
    localVel = speed;
}
