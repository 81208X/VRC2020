#include "controller.hpp"
#include <array>

pros::task_t callbackTask;
std::array<std::function<void()>, 12> callbacks;
pros::controller_id_e_t controllerId;
uint32_t curTime;
void callbackTaskFn(void*) {
    // pros::Controller master(*(pros::Controller*)controllerPtr);
    bool pressed[pros::E_CONTROLLER_DIGITAL_A - 5] = {};
    while(true) {
        // HACK: this cast should never really be done for enum values, but its fine since they are contiguous and I dont overflow it
        for(pros::controller_digital_e_t btn = pros::E_CONTROLLER_DIGITAL_L1; btn < pros::E_CONTROLLER_DIGITAL_A; btn = static_cast<pros::controller_digital_e_t>(static_cast<int>(btn) + 1)) {
            if(pros::c::controller_get_digital(controllerId, btn) && !pressed[btn-6]) {
                pressed[btn-6] = true;
                if(callbacks[btn-6]) // Make sure there is a valid function here
                                     // Its really cool that std::function casts to a bool in this way, super clean
                    callbacks[btn-6]();
            }
            else
                pressed[btn-6] = false;
        }
        curTime = pros::millis();
        pros::Task::current().delay_until(&curTime, POLL_INTERVAL); // I think this is a pointer to memory on the stack, probably fine tho
    }
}

Controller::Controller(pros::controller_id_e_t controller) : master(controller) {
    controllerId = controller;
}

bool Controller::getBtnRaw(pros::controller_digital_e_t button) {
    return master.get_digital(button);
}

bool Controller::getBtnNew(pros::controller_digital_e_t button) {
    return master.get_digital_new_press(button);
}

bool Controller::rumble(const char* rumblePattern) {
    // TODO: handle fast/repeated calling
    // Maybe use mutex here?
    return (bool)master.rumble(rumblePattern);
}

bool Controller::print(const char* message) {
    return (bool)master.print(1,1,message);
}

int32_t Controller::getAxis(pros::controller_analog_e_t axis) {
    return master.get_analog(axis);
}

void Controller::enableCallbacks() {
    callbackTask = pros::c::task_create(callbackTaskFn, (void*)NULL, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Controller callback task");
}

void Controller::disableCallbacks() {
    pros::c::task_delete(callbackTask);
}

void Controller::registerCallback(pros::controller_digital_e_t btn, std::function<void()> cb) {
    callbacks[btn - pros::E_CONTROLLER_DIGITAL_L1] = cb;
}

void Controller::unregisterCallbacks() {
    callbacks = {};
}
