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
        , controller(controller) {};
    bool run() override {
        for (auto& i : digital_buttons) {
            vc.set_button(i, false);
        }
        return true;
    }

  private:
    virtualController& vc;
    pros::Controller& controller;
};
class update_controller_value_c : public command {
  public:
    explicit update_controller_value_c(virtualController& vc,
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