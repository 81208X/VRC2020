#include "indexer.hpp"

#include "okapi/api.hpp"
#include "subsystem.hpp"

/* ---------- INDEXER DEBUG FUNCTIONS ---------- */
// These functions overide instructions from the rest of this wrapper
// Do not reference in competition code!
void Indexer::debugIn() {
    intake.moveVelocity(-200);
    lowerRoller.moveVelocity(-100);
    upperRoller.moveVelocity(-50);
}

void Indexer::debugOut() {
    intake.moveVelocity(200);
    lowerRoller.moveVelocity(300);
    upperRoller.moveVelocity(50);
}

void Indexer::debugStop() {
    intake.moveVelocity(0);
    rollerMtrGrp.moveVelocity(0);
}

/* ---------- INDEXER CONTROL METHODS ---------- */
bool Indexer::checkTimeout(uint32_t timeout, uint32_t delay) {
    pros::delay(delay);
    if((timeout + taskStartTime) < pros::millis()) {
        intake.moveVelocity(0);
        upperRoller.moveVelocity(0);
        lowerRoller.moveVelocity(0);
        return true;
    } else
        return false;
}

void Indexer::getUpperBall(uint32_t timeout) {
    taskStartTime = pros::millis();

    if(gotUpperBall)
        return;

    intake.moveVelocity(-200);
    lowerRoller.moveVelocity(-300);
    upperRoller.moveVelocity(-200);
    
    if(!gotLowerBall)
        while(frontIndexer.get_value() > INDEXER_FRONT_DETECTION_THRESHOLD &&
              backIndexer.get_value() > INDEXER_BACK_DETECTION_THRESHOLD)
        {
            if(checkTimeout(timeout))
                return;
        }

    intake.moveVelocity(0);

    while(backIndexer.get_value() > INDEXER_BACK_DETECTION_THRESHOLD)
        if(checkTimeout(timeout))
            return;

    while(backIndexer.get_value() < INDEXER_BACK_DETECTION_THRESHOLD2)
        if(checkTimeout(timeout))
            return;
    pros::delay(150);

    gotLowerBall = false;
    gotUpperBall = true;
    upperRoller.moveVelocity(0);
    lowerRoller.moveVelocity(0);
}

void Indexer::getLowerBall(uint32_t timeout) {
    taskStartTime = pros::millis();
    if(gotLowerBall)
        return;

    intake.moveVelocity(-200);
    lowerRoller.moveVelocity(-250);

    while(!gotLowerBall) {
        if(frontIndexer.get_value() < INDEXER_FRONT_DETECTION_THRESHOLD)
            gotLowerBall = true;
        if(checkTimeout(timeout))
            return;
    }
    intake.moveVelocity(0);
    lowerRoller.moveVelocity(0);
}

void Indexer::getIntakeBall(uint32_t timeout) {
    bool gotIntakeBall = false;
    intake.moveVelocity(-100);
    while(!gotIntakeBall) {
        if(visionSensorLower.sensor.get_object_count() > 0) {
            pros::vision_object_s_t obj = visionSensorLower.sensor.get_by_size(0);
            if(obj.top_coord > VISION_LOWER_INTAKE_MIN_Y_POS &&
               obj.width > VISION_LOWER_INTAKE_MIN_X_POS)
            {
                gotIntakeBall = true;
            }
        }
        if(checkTimeout(timeout, 20))
            return;
    }
}

void Indexer::score(double speed) {
    speed = std::clamp(speed, 0.0, 1.0);
    if(gotUpperBall) {
        upperRoller.moveRelative(-1200, 600*speed);
        gotUpperBall = false;
    }
    else if(gotLowerBall) {
        lowerRoller.moveRelative(-1200, 600*speed);
        upperRoller.moveRelative(-2400, 600*speed);
        gotLowerBall = false;
    }
}

void Indexer::discardLowerBall() {
    intake.moveVelocity(200);
    lowerRoller.moveVelocity(400);
    pros::delay(600);
    intake.moveVelocity(0);
    lowerRoller.moveVelocity(0);
    gotLowerBall = false;
}

void Indexer::getBothBall(uint32_t timeout) {
    uint32_t firstStartTime = pros::millis();
    getUpperBall();
    timeout = timeout + (firstStartTime - pros::millis());
    if(timeout < 0)
        return;
    getLowerBall();
}

void Indexer::getAllBalls(uint32_t timeout) {
    uint32_t firstStartTime = pros::millis();
    getBothBall(timeout);
    timeout = timeout + (firstStartTime - pros::millis());
    if(timeout < 0)
        return;
    getIntakeBall();
}
