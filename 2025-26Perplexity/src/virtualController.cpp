#include "virtualController.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include <map>

void virtualController::init_buttons() {
    for (auto& i : analog_buttons) {
        buttons[i] = false;
    }
}

void virtualController::update_controller(pros::Controller controller) {
    for (auto& i : analog_buttons) {
        if (buttons[i] != controller.get_digital(i)) {
            buttons[i] = !buttons[i];
        }
    }
}