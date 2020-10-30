#include "util.hpp"
#include <algorithm>
#include "profiles.hpp"
#include "io.hpp"

util::SlewRateLimiter::SlewRateLimiter(double rateLimit, double initValue, double rateOnDecel){
    this->ratelimit = rateLimit;
    decelRate = rateOnDecel==0 ? rateLimit : rateOnDecel;
    reset(initValue);
}

double util::SlewRateLimiter::calculate(double newValue){
    double timeDiff = (pros::millis() - lastTime) / 1000.0;
    if(abs(newValue) > abs(lastValue)){
        lastValue = std::clamp(newValue, lastValue - ratelimit * timeDiff, lastValue + ratelimit * timeDiff);
    }else{
        lastValue = std::clamp(newValue, lastValue - decelRate * timeDiff, lastValue + decelRate * timeDiff);
    }
    lastTime = pros::millis();
    return lastValue;
}

void util::SlewRateLimiter::reset(double newValue){
    lastTime = pros::millis();
    lastValue = newValue;
}

bool util::runAsBlocking(std::function<void()> fn, std::function<bool()> condition) {
    fn();
    return blocking(condition);
}

bool util::runAsBlocking(std::function<void()> fn, std::function<bool()> condition, int timeout, int pollRate) {
    fn();
    return blocking(condition, timeout, pollRate);
}

bool util::blocking(std::function<bool()> condition) {
    return blocking(condition, NO_TIME_OUT, BLOCKING_DEFAULT_POLL_RATE);
}

bool util::blocking(std::function<bool()> condition, int timeOut, int pollRate) {
    pros::delay(5);
    uint32_t time, startTime = pros::millis();
    while (!condition()) {
        time = pros::millis();
        if (timeOut != NO_TIME_OUT && (time - startTime) > (uint32_t)timeOut)
            return false;
        pros::Task::delay_until(&time, pollRate);
    }
    pros::delay(5);
    return true;
}
