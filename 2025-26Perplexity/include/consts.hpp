#pragma once

#include "pros/misc.h"
#include <list>
enum Outt_States { OFF = 0, TOP = 1, BOTTOM = 2, MIDDLE = 3, HOARD = 4 };

struct poseCFG {
    bool reversed = false;
    int minSpeed = 0;
    int maxSpeed = 120;
    int timeout = 6000;
    float earlyExitRange = 1;
};
const std::list<pros::controller_digital_e_t> digital_buttons = {
    pros::E_CONTROLLER_DIGITAL_A,    pros::E_CONTROLLER_DIGITAL_L1,
    pros::E_CONTROLLER_DIGITAL_R2,   pros::E_CONTROLLER_DIGITAL_RIGHT,
    pros::E_CONTROLLER_DIGITAL_X,    pros::E_CONTROLLER_DIGITAL_Y,
    pros::E_CONTROLLER_DIGITAL_L2,   pros::E_CONTROLLER_DIGITAL_LEFT,
    pros::E_CONTROLLER_DIGITAL_R1,   pros::E_CONTROLLER_DIGITAL_UP,
    pros::E_CONTROLLER_DIGITAL_DOWN, pros::E_CONTROLLER_DIGITAL_B
};

struct box {
    float minX;
    float minY;
    float maxX;
    float maxY;
};