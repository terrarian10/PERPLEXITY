#include "commandHandling.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "pros/misc.h"
#include "virtualController.hpp"

class tank_c : command {
  public:
    explicit tank_c(lemlib::Chassis& chassis, virtualController& vc)
        : chassis(chassis)
        , vc(vc) {};
    bool run() override {
        chassis.tank(vc.get_joystick(pros::E_CONTROLLER_ANALOG_LEFT_Y),
                     vc.get_joystick(pros::E_CONTROLLER_ANALOG_RIGHT_Y));
        return true;
    };

  private:
    lemlib::Chassis& chassis;
    virtualController& vc;
};