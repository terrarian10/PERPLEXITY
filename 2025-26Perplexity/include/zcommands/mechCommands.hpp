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