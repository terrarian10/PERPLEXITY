#include "Outtake.hpp"
#include "airCylinder.hpp"
#include "commandHandling.hpp"
#include "consts.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "main.h"
#include "pros/misc.h"
#include <unordered_map>

class mech_state_c : public command {
  public:
    explicit mech_state_c(pros::controller_digital_e_t state, Outtake& outtake)
        : outtake(outtake)
        , state(state) {};
    bool run() override {
        outtake.run_at_state(state);
        return true;
    };

  private:
    Outtake& outtake;
    pros::controller_digital_e_t state;
};
class update_mech_state : public command {
  public:
    explicit update_mech_state(Outtake& outtake,
                               mecha_control& states,
                               virtualController& vc)
        : outtake(outtake)
        , states(states)
        , vc(vc) {};
    bool run() override {
        auto presses = vc.getNewPress();
        for (auto& i : states.motorHandling) {
            if (std::find(presses.begin(), presses.end(), i.control) !=
                presses.end()) {
                if (outtake.get_state_ctrl() == i.control &&
                    outtake.getRunning()) {
                    outtake.halt();
                } else {
                    outtake.run_at_state(i.control);
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
class update_mech : public command {
  public:
    explicit update_mech(Outtake& outtake)
        : outtake(outtake) {};
    bool run() override {
        outtake.loop();
        return true;
    };

  private:
    Outtake& outtake;
};
class togglePneu_c : public command {
  public:
    explicit togglePneu_c(AirCylinder& air)
        : air(air) {};
    bool run() override {
        air.toggle();
        return true;
    }
    void quit() override {}
    void force_quit() override {}

  private:
    AirCylinder& air;
};
class switchPneu_c : public command {
  public:
    explicit switchPneu_c(AirCylinder& air, bool state)
        : air(air)
        , state(state) {};
    bool run() override {
        air.set_value(state);
        return true;
    }
    void quit() override {}
    void force_quit() override {}

  private:
    AirCylinder& air;
    bool state;
};
class update_pneu : public command {
  public:
    explicit update_pneu(std::vector<AirCylinder*> airs,
                         std::vector<pros::controller_digital_e_t> pneuCtrl,
                         virtualController& vc)
        : airs(std::move(airs))
        , pneuCtrl(std::move(pneuCtrl))
        , vc(vc) {}

    bool run() override {
        const auto& r = vc.getNewPress(); // ideally returns const ref
        const size_t n = std::min(airs.size(), pneuCtrl.size());

        for (size_t i = 0; i < n; i++) {
            if (std::find(r.begin(), r.end(), pneuCtrl[i]) != r.end()) {
                airs[i]->toggle();
            }
        }
        return true;
    }

  private:
    std::vector<AirCylinder*> airs;
    std::vector<pros::controller_digital_e_t> pneuCtrl;
    virtualController& vc;
};
