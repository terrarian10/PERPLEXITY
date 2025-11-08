#include "pros/misc.h"
#include "pros/misc.hpp"
#include <list>
#include <map>

class virtualController {
  public:
    // virtualController();
    std::map<pros::controller_digital_e_t, bool> buttons;
    inline void set_button(pros::controller_digital_e_t i, bool value) {
        buttons[i] = value;
    };
};