#ifndef _UTIL_HPP_INCLUDED
#define _UTIL_HPP_INCLUDED

#include <cmath>
#include <functional>
#include "api.h"
#include "okapi/api.hpp"
#include "util/struct.hpp"

/* Math constants incase they weren't defined for some reason */
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifndef M_PI_2
#define M_PI_2 1.57079632679489661923
#endif

#ifndef M_SQRT2
#define M_SQRT2 1.41421356237309504880
#endif

#define BLOCKING_DEFAULT_POLL_RATE 30
#define NO_TIME_OUT -1

namespace util {

    /** 
     * Helps in limitting the rate of change of a certain value.
     * A different rate limit can be set for when the absolute value of the target value is decreasing.
     * This can be useful when you want the robot to speed up and slow down at different rates, for example.
     */
    class SlewRateLimiter{
    private:
        double ratelimit;
        double lastValue;
        double decelRate;
        uint32_t lastTime;
        
    public:
        /**
         * Helps in limitting the rate of change of a certain value.
         * A different rate limit can be set for when the absolute value of the target value is decreasing.
         * This can be useful when you want the robot to speed up and slow down at different rates, for example.
         * 
         * \param rateLimit The maximum change the target value can have in 1 second.
         * 
         * \param initValue The initial value for the target value. Defaults to 0
         * 
         * \param rateOnDecel A different rate for when the absolute value of the target value is decreasing.
         * Defaults to the same as rateLimit.
         */
        SlewRateLimiter(double rateLimit, double initValue = 0, double rateOnDecel = 0);

        /**
         * Calculate what the new value should be after applying the rate limits.
         * 
         * \param newValue the targetted value.
         * 
         * \return The value after getting rate limitted.
         */
        double calculate(double newValue);

        /**
         * Sets the current value, bypassing the rate limit.
         * 
         * \param newValue The value to be set as the current value.
         */
        void reset(double newValue);
    };

    ///Returns the average of 2 double numbers
    inline double avgDouble(double a, double b) {
        return (a + b) / 2.0;
    };

    //Given an angle in radians, returns the smallest equivalent angle.
    inline double wrapAngle(double angle) {
        return angle - (M_PI * 2) * std::floor((angle + M_PI) / (M_PI * 2));
    };

    //Given an angle in radians, returns the smallest angle that's either equivalent or opposite to the current one.
    inline double wrapAngle90(double angle) {
        double a = wrapAngle(angle);
        if(abs(a)>M_PI/2.0){
            a+=M_PI;
            a = wrapAngle(a);
        }
        return a;
    };

    /**
     * Runs an asynchronous function as blocking.
     * \param fn the asynchronous function to run.
     * 
     * \param condition A function that returns true if the asynchronous task has finished, false otherwise.
     * The runAsBlocking function will block for as long as condition returns false.
     * 
     * \param timeoutMs A timeout amount in milliseconds. Stops blocking after a certain amount of time.
     * 
     * \param pollRate The rate at which the condition is checked.
     */
    bool runAsBlocking(std::function<void()> fn, std::function<bool()> condition, int timeoutMs, int pollRate = BLOCKING_DEFAULT_POLL_RATE);

    /**
     * Runs an asynchronous function as blocking.
     * \param fn the asynchronous function to run.
     * 
     * \param condition A function that returns true if the asynchronous task has finished, false otherwise.
     * The runAsBlocking function will block for as long as condition returns false.
     */
    bool runAsBlocking(std::function<void()> fn, std::function<bool()> condition);

    /**
     * Blocks the current task until a condition function returns true.
     * 
     * \param condition A function that returns true to allow the code to continue executing, false otherwise.
     * The blocking function will block for as long as condition returns false.
     * 
     * \param timeoutMs A timeout amount in milliseconds. Stops blocking after a certain amount of time.
     * 
     * \param pollRate The rate at which the condition is checked.
     */
    bool blocking(std::function<bool()>, int timeOutMs, int pollRateMs = BLOCKING_DEFAULT_POLL_RATE);

    /**
     * Blocks the current task until a condition function returns true.
     * 
     * \param condition A function that returns true to allow the code to continue executing, false otherwise.
     * The blocking function will block for as long as condition returns false.
     */
    bool blocking(std::function<bool()>);

    /**
     * Runs a blocking function as asynchronous in its own pros task.
     * 
     * \param fn the function to be run asynchronously.
     * 
     * \return The task that's created
     */
    inline pros::Task runAsync(std::function<void()> fn) {
        return pros::Task(fn, "Async task");
    };
}
#endif /* _UTIL_HPP_INCLUDED */
