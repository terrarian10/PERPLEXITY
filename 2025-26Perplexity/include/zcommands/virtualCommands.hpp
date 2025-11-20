#include "commandHandling.hpp"
#include "consts.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "virtualController.hpp"

class init_controller_c : public command {
  public:
    explicit init_controller_c(virtualController& vc)
        : vc(vc) {};
    bool run() override {
        for (auto& i : digital_buttons) {
            vc.set_button(i, false);
        }
        return true;
    }

  private:
    virtualController& vc;
};

class update_controller_c : public command {
  public:
    explicit update_controller_c(virtualController& vc,
                                 pros::Controller& controller)
        : vc(vc)
        , controller(controller) {}

    bool run() override {
        // digital buttons
        for (auto& i : digital_buttons) {
            vc.set_button(i, controller.get_digital(i));
        }

        // joysticks
        vc.set_joystick(
            pros::E_CONTROLLER_ANALOG_LEFT_Y,
            controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));
        vc.set_joystick(
            pros::E_CONTROLLER_ANALOG_RIGHT_Y,
            controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y));
        vc.set_joystick(
            pros::E_CONTROLLER_ANALOG_LEFT_X,
            controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X));
        vc.set_joystick(
            pros::E_CONTROLLER_ANALOG_RIGHT_X,
            controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));

        // continuous “always running” command → don't finish
        return false;
    }

  private:
    virtualController& vc;
    pros::Controller& controller;
};

class set_controller_button_c : public command {
  public:
    explicit set_controller_button_c(virtualController& vc,
                                     pros::controller_digital_e_t item,
                                     bool value)
        : vc(vc)
        , item(item)
        , value(value) {};
    bool run() override {

        vc.set_button(item, value);

        return true;
    }

  private:
    virtualController& vc;
    bool value;
    pros::controller_digital_e_t item;
};
class set_controller_analog_c : public command {
  public:
    explicit set_controller_analog_c(virtualController& vc,
                                     pros::controller_analog_e_t item,
                                     float value)
        : vc(vc)
        , item(item)
        , value(value) {};
    bool run() override {

        vc.set_joystick(item, value);

        return true;
    }

  private:
    virtualController& vc;
    float value;
    pros::controller_analog_e_t item;
};
