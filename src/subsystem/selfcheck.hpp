#ifndef _SELFCHECK_HPP_INCLUDED
#define _SELFCHECK_HPP_INCLUDED

#include <vector>
#include "api.h"


class SelfCheck {
private:
    pros::task_t selfCheckTask;
    bool taskRunning = false;
    
public:
    /// A list of all V5 smart ports that should be connected to a motor
    std::vector<int> ports;

    /**
     * Initializes self check with the list of motor ports.
     * The self check task, when started, will report the highest temperature motor to the driver's controller.
     * If any motor is disconnected or overheating, the driver will be noticed with a text message and controller vibrations.
     * 
     * \param portsUsed A list of all V5 smart ports that should be connected to a motor
     */
    SelfCheck(std::vector<int> portsUsed);

    /**
     * Start the asynchronous self check task
     * 
     * The self check task, when started, will report the highest temperature motor to the driver's controller.
     * If any motor is disconnected or overheating, the driver will be noticed with a text message and controller vibrations.
     */
    void startTask();

    ///Ends the asynchronous self check task
    void endTask();
};
#endif /* _SELFCHECK_HPP_INCLUDED */
