#include "pros/misc.h"
#include "pros/misc.hpp"
#include <list>
#include <map>

class virtualController {
  public:
    // virtualController();
    std::map<pros::controller_digital_e_t, bool> buttons;
    void init_buttons();
    void update_controller(pros::Controller controller);

  private:
    std::list<pros::controller_digital_e_t> analog_buttons = {
        pros::E_CONTROLLER_DIGITAL_A,    pros::E_CONTROLLER_DIGITAL_L1,
        pros::E_CONTROLLER_DIGITAL_R2,   pros::E_CONTROLLER_DIGITAL_RIGHT,
        pros::E_CONTROLLER_DIGITAL_X,    pros::E_CONTROLLER_DIGITAL_Y,
        pros::E_CONTROLLER_DIGITAL_L2,   pros::E_CONTROLLER_DIGITAL_LEFT,
        pros::E_CONTROLLER_DIGITAL_R1,   pros::E_CONTROLLER_DIGITAL_UP,
        pros::E_CONTROLLER_DIGITAL_DOWN, pros::E_CONTROLLER_DIGITAL_B
    };
};