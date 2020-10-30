#ifndef _CONTROLLER_HPP_INCLUDED
#define _CONTROLLER_HPP_INCLUDED

#include "api.h"
#include <functional>

#define POLL_INTERVAL 10

class Controller {
private:
    pros::Controller master;
    
public:
    Controller(pros::controller_id_e_t);

    /**
	 * Registers a callback to a button on the controller.
     *
     * \param btn
     *        The button to register the callback to.
     * \param callback
     *        The function to callback.
	 */
    void registerCallback(pros::controller_digital_e_t, std::function<void()>);

    /// Unregisters all active callbacks.
    void unregisterCallbacks();

    /// Enable the callback system, start the callback polling task.
    void enableCallbacks();

    /// Disable the callback system, stop the callback polling task if active.
    void disableCallbacks();

    /**
	 * Gets the value of a button on the controller.
     *
     * \param btn
     *        The button to check the value of.
     *
     * \return True if the button is pressed, false otherwise.
	 */
    bool getBtnRaw(pros::controller_digital_e_t);

    /**
	 * Checks if a button has been pressed since last call to this function.
     * Does not handle concurrency well so only call from a single loop.
     *
     * \param btn
     *        The button to check the value of.
     *
     * \return True if the button has been pressed since the last check,
     * false otherwise.
	 */
    bool getBtnNew(pros::controller_digital_e_t);

    /**
     * Gets the value of an analog axis on the controller.
     *
     * \return The value of the analog axis, ranging from -127 to 127.
     */
    int32_t getAxis(pros::controller_analog_e_t);

    /**
     * Rumbles the controller with the specified pattern.
     *
     * \param pattern
     *        The pattern to vibrate the controller with.
     *        See PROS documentation for pattern syntax.
     *
     * \return True if successful, false if not.
     */
    bool rumble(const char*);

    /**
     * Prints a string to the controller display.
     *
     * \param string
     *        The string to print to the controller.
     *
     * \return True if successful, false if not.
     */
    bool print(const char*);
};
#endif /* _CONTROLLER_HPP_INCLUDED */
