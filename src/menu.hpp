#ifndef _MENU_HPP_INCLUDED
#define _MENU_HPP_INCLUDED

#include "okapi/api.hpp"

// The total number of menu items
#define MENU_LIMIT 7

class Menu {
private:
    pros::task_t menuTask, screensaverTask;
    int menuSelected = 0;
    bool taskRunning = false, graphicsRunning = false;
    // Previous values for buttons. Used for detecting new presses.
    bool lastOk, lastL, lastR;
    static void graphicsTask(void*);
    static void menuTaskFn(void*);
    
public:
    /// Wether or not to intercept controller inputs for menu navigation.
    bool controllerNavigation = false;
    Menu();

    /// Starts the menu task
    void startTask();
    
    /// Ends the menu task
    void endTask();

    /// an accessor function for the menuSelected variable. Returns the index of the currently selected menu page.
    int getCurrentMenuId();
    
    /// Returns wether or not the OK button is pressed. This function switches between controller and LCD button automatically.
    bool getRawOkBtn();
    
    /// Returns wether or not the OK button was just pressed. This function switches between controller and LCD button automatically.
    bool getNewOkBtn();

    /// Updates the Left and Right button press values. Changes the current menu page on button presses.
    void handleMenuNavigation();

    /// DVD bouncing logo inspired bouncing logo!
    void initGraphics();
};
#endif /* _MENU_HPP_INCLUDED */
