#include "pros/misc.h"
#include "pros/misc.hpp"
#include <list>
#include <map>

class virtualController {
  public:
    // virtualController();
    std::map<pros::controller_digital_e_t, bool> buttons;
    std::map<pros::controller_analog_e_t, float> joysticks;
    void set_button(pros::controller_digital_e_t i, bool value) {
        buttons[i] = value;
    };
    void set_joystick(pros::controller_analog_e_t i, float value) {
        joysticks[i] = value;
    };
    bool get_button(pros::controller_digital_e_t i) const {
        auto it = buttons.find(i);
        return it != buttons.end() ? it->second : false;
    }

    int get_joystick(pros::controller_analog_e_t i) const {
        auto it = joysticks.find(i);
        return it != joysticks.end() ? it->second : 0;
    }
};