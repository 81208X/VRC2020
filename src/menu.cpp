#include "menu.hpp"

#include <experimental/random>
#include "pros/apix.h"
#include "systemmanager.hpp"
#include "profiles.hpp"

#ifdef LOGO_NAME
LV_IMG_DECLARE(LOGO_NAME);
#endif

Menu* theMenu;

Menu::Menu() {
    theMenu = this;
}

void Menu::menuTaskFn(void *param) {
    char temp[100];
    Menu *menu = (Menu *)param;
    // Print auton and side selection to controller
    // Placeholder _ is to workaround os bug with quickly refreshing controller
    sprintf(temp, "A: %d, S: %s", robotConfigs.auton, robotConfigs.autonSide & LocalStorage::AutonMode::RED ? "red_" : "blue");
    controllerMaster.print(temp);
    while (true) {
        // menu rollover
        menu->handleMenuNavigation();
        switch (menu->getCurrentMenuId()) {
            case 0: // Toggle controller menu control
                pros::lcd::print(2, "\t> %u: Controller Menu Mode (%u)", menu->getCurrentMenuId(), menu->controllerNavigation);
                pros::lcd::print(3, "\t(disables opcontrol when set)");
                if (menu->getNewOkBtn())
                    menu->controllerNavigation = !menu->controllerNavigation;
                break;
            case 1: // start of blue and red side selection
                pros::lcd::print(2, "\t> %u: Side Toggle", menu->getCurrentMenuId());
                pros::lcd::print(3, "\t    %s", robotConfigs.autonSide & LocalStorage::AutonMode::RED ? "Red" : "Blue");
                if (menu->getNewOkBtn()) {
                    robotConfigs.autonSide = (LocalStorage::AutonMode)(robotConfigs.autonSide ^ LocalStorage::AutonMode::RED);
                    localStorage.writeConfigs();
                    sprintf(temp, "A: %d, S: %s", robotConfigs.auton, robotConfigs.autonSide & LocalStorage::AutonMode::RED ? "red_" : "blue");
                    controllerMaster.print(temp);
                }
                break; // end of blue and red side selection
            case 2: // start of top/bottom side selection
                pros::lcd::print(2, "\t> %u: Side Toggle (for backup)", menu->getCurrentMenuId());
                pros::lcd::print(3, "\t    %s", robotConfigs.autonSide & LocalStorage::AutonMode::TOP ? "Top" : "Bottom");
                if (menu->getNewOkBtn()) {
                    // robotConfigs.topSide = !robotConfigs.topSide;
                    robotConfigs.autonSide = (LocalStorage::AutonMode)(robotConfigs.autonSide ^ LocalStorage::AutonMode::TOP);
                    localStorage.writeConfigs();
                }
                break; // end of top/bottom side selection
            case 3:  // start of auton path selection
                pros::lcd::print(2, "\t> %u: Auton PATH Selection", menu->getCurrentMenuId());
                switch (robotConfigs.auton) {
                    // TODO: Update for 2020 auton paths when finalized
                    case -1:
                        pros::lcd::print(3, "\t    -1 auton");
                        break;
                    case 0:
                        pros::lcd::print(3, "\t    Default");
                        break;
                    default:
                        pros::lcd::print(3, "\t    INVALID AUTON MODE");
                }
                if (menu->getNewOkBtn()) {
                    robotConfigs.auton = robotConfigs.auton >= 4 ? -1 : robotConfigs.auton + 1;
                    localStorage.writeConfigs();
                    sprintf(temp, "A: %d, S: %s", robotConfigs.auton, robotConfigs.autonSide & LocalStorage::AutonMode::RED ? "red_" : "blue");
                    controllerMaster.print(temp);
                }
                break; // end of auton path selection
            case 4:
                pros::lcd::print(2, "\t> %u: Skills Mode Toggle", menu->getCurrentMenuId());
                pros::lcd::print(3, "\t    %s", robotConfigs.driverSkills ? "Driver Skills" : "Default");
                if (menu->getNewOkBtn()) {
                    robotConfigs.driverSkills = !robotConfigs.driverSkills;
                    localStorage.writeConfigs();
                }
                break;
            case 5:
                pros::lcd::print(2, "\t> %u: \"Competition\" Toggle", menu->getCurrentMenuId());
                pros::lcd::print(3, "\t    %s", robotConfigs.debugging ? "Debugging" : "Competition");
                if (menu->getNewOkBtn()) {
                    robotConfigs.debugging = !robotConfigs.debugging;
                    localStorage.writeConfigs();
                    pros::lcd::clear();
                }
                break;
            case 6:
                pros::lcd::print(2, "\t> %u: Logging Toggle", menu->getCurrentMenuId());
                pros::lcd::print(3, "\t    Logging %s", robotConfigs.loggingEnable ? "enabled" : "disabled");
                if (menu->getNewOkBtn()) {
                    robotConfigs.loggingEnable = !robotConfigs.loggingEnable;
                    localStorage.writeConfigs();
                }
                break;
            case 7:
                pros::lcd::print(2, "\t> %u: Enable graphics, disable menu", menu->getCurrentMenuId());
                pros::lcd::clear_line(3);
                if (menu->getNewOkBtn()) {
                    menu->initGraphics();
                    menu->controllerNavigation = false;
                }
                break;
        }
        pros::delay(20);
    }
}

void Menu::handleMenuNavigation() {
    bool leftSelected = pros::c::lcd_read_buttons()&LCD_BTN_LEFT;
    bool rightSelected = pros::c::lcd_read_buttons()&LCD_BTN_RIGHT;

    if (controllerNavigation) {
        leftSelected |= controllerMaster.getBtnRaw(pros::E_CONTROLLER_DIGITAL_LEFT);
        rightSelected |= controllerMaster.getBtnRaw(pros::E_CONTROLLER_DIGITAL_RIGHT);
    }

    if (leftSelected) {
        if(!lastL && menuSelected > 0)
        menuSelected--;
    }
    else if (rightSelected && menuSelected < MENU_LIMIT) {
        if (!lastR)
            menuSelected++;
    }

    lastL = leftSelected;
    lastR = rightSelected;
}

bool Menu::getNewOkBtn() {
    if (robotConfigs.debugging)
        pros::lcd::print(4, "\t%d",pros::c::lcd_read_buttons());
    bool ok = pros::c::lcd_read_buttons()&LCD_BTN_CENTER;
    if(controllerNavigation)
        ok |= controllerMaster.getBtnRaw(pros::E_CONTROLLER_DIGITAL_A);
    if (ok) {
        if(!lastOk) {
            lastOk = true;
            return true;
        }
        return false;
    } else {
        lastOk = false;
        return false;
    }
}

bool Menu::getRawOkBtn() {
    lastOk = pros::c::lcd_read_buttons()&LCD_BTN_CENTER;
    if(controllerNavigation)
        lastOk |= controllerMaster.getBtnRaw(pros::E_CONTROLLER_DIGITAL_A);
    return lastOk;
}

void Menu::startTask() {
    if (taskRunning || graphicsRunning)
        endTask();
    menuTask = pros::c::task_create(menuTaskFn, this, TASK_PRIORITY_DEFAULT,
                                    TASK_STACK_DEPTH_DEFAULT, "Menu Task");
    taskRunning = true;
}
void Menu::endTask() {
    controllerNavigation = false;
    if (taskRunning) {
        taskRunning = false;
        pros::c::task_delete(menuTask);
    }
    if (graphicsRunning) {
        graphicsRunning = false;
        pros::c::task_delete(screensaverTask);
    }
}

int Menu::getCurrentMenuId() {
    return menuSelected;
}

lv_res_t restartTaskCb(lv_obj_t * btn) {
    lv_obj_del(btn);
    // pros::lcd::initialize();
    theMenu->startTask();
    theMenu->controllerNavigation = true;
    return LV_RES_OK;
}

void Menu::initGraphics() {
    #if !defined LOGO_NAME || defined DISABLE_LOGO
    return;
    #endif
    if(graphicsRunning)
        return;
    lv_obj_t * scr = lv_scr_act();
    // lv_obj_clean(scr);
    lv_obj_t * btn =  lv_btn_create(scr, NULL);
    lv_btn_set_fit(btn, true, true);                        /*Automatically increase the size to involve all children (now only the image)*/
    lv_btn_set_style(btn, LV_BTN_STYLE_REL, &lv_style_transp_tight);   /*Transparent style for released button*/
    lv_btn_set_style(btn, LV_BTN_STYLE_PR, &lv_style_transp_tight);   /*Transparent style for pressed button*/
    lv_btn_set_action(btn, LV_BTN_ACTION_CLICK, restartTaskCb);

    lv_obj_t * logo_var = lv_img_create(btn, NULL); /*Crate an image object*/
    lv_obj_set_click(logo_var, false);
    lv_img_set_src(logo_var, &LOGO_NAME);  /*Set the created file as image*/
    // lv_obj_set_pos(btn, std::rand() % (lv_obj_get_width(scr) - lv_obj_get_width(logo_var)), std::rand() % (lv_obj_get_height(scr) - lv_obj_get_height(logo_var)));
    lv_obj_set_pos(btn, std::experimental::randint(0, lv_obj_get_width(scr) - lv_obj_get_width(logo_var)), std::experimental::randint(0, lv_obj_get_height(scr) - lv_obj_get_height(logo_var)));
    // lv_obj_set_opa_scale_enable(logo_var, true);
    // lv_obj_set_opa_scale(logo_var, 2);
    // lv_obj_set_drag(logo_var, true);
    graphicsRunning = true;
    screensaverTask = pros::c::task_create(graphicsTask, btn, TASK_PRIORITY_MIN, TASK_STACK_DEPTH_DEFAULT, "Graphics animation task");
}

void Menu::graphicsTask(void* ptr) {
    float xspeed = 2, yspeed = 1;
    bool xdirection = std::experimental::randint(0, 1), ydirection = std::experimental::randint(0, 1);
    lv_obj_t * logo_var = (lv_obj_t *)ptr;
    lv_coord_t w = lv_obj_get_width(logo_var);
    lv_coord_t h = lv_obj_get_height(logo_var);
    lv_coord_t scr_w = lv_obj_get_width(lv_scr_act());
    lv_coord_t scr_h = lv_obj_get_height(lv_scr_act());
    uint32_t time;
    while(true) {
        lv_coord_t x = lv_obj_get_x(logo_var);
        lv_coord_t y = lv_obj_get_y(logo_var);

        if (xdirection)
            x += (int)xspeed;
        else
            x -= (int)xspeed;
        if (ydirection)
            y += (int)yspeed;
        else
            y -= (int)yspeed;

        // Test to see if the shape exceeds the boundaries of the screen
        // If it does, reverse its direction by multiplying by -1
        if (x > scr_w-w) {
            xdirection = 0;
            x = scr_w-w;
        }
        else if (x <= 0) {
            xdirection = 1;
            x = 0;
        }
        if (y > scr_h-h) {
            ydirection = 0;
            y = scr_h-h;
        }
        else if (y <= 0) {
            ydirection = 1;
            y = 0;
        }

        lv_obj_set_pos(logo_var, x, y);

        time = pros::millis();
        pros::Task::delay_until(&time, 17); // ~60fps
    }
}
