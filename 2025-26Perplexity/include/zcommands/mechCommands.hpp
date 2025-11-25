#include "Outtake.hpp"
#include "commandHandling.hpp"
#include "consts.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "pros/misc.h"
#include "virtualController.hpp"

class mech_state_c : command {
  public:
    explicit mech_state_c(Outtake& outtake, Outt_States state)
        : outtake(outtake)
        , state(state) {};
    bool run() override {
        outtake.run_at_state(state);
        return true;
    };

  private:
    Outtake& outtake;
    Outt_States state;
};
class update_mech_state : command {
  public:
    explicit update_mech_state(Outtake& outtake,
                               mecha_control& states,
                               virtualController& vc)
        : outtake(outtake)
        , states(states)
        , vc(vc) {};
    bool run() override {
        for (auto& i : states.motorHandling) {
            if (vc.get_button(i.control)) {
                if (outtake.get_state() == i.id) {
                    outtake.run_at_state(Outt_States::OFF);
                } else {
                    outtake.run_at_state(i.id);
                }
            }
        }
        return true;
    };

  private:
    Outtake& outtake;
    mecha_control& states;
    virtualController& vc;
};
