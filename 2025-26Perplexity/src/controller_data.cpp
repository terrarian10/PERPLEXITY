#include "controller_data.hpp"
#include "Outtake.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include <functional>
#include <iterator>
#include <vector>

void ModularControl::updateDisplay(ModularComponent modc) {
    controller.clear();
    if (modc == ModularComponent::MATCHLOAD) {
        controller.set_text(1, 0, ("MATCHLOAD"));
    }
}

// I HAVE DECIDED ON THE MOST STUPID CONVOLUTED OVERPOWERED OVERENGINEERED MACRO
// SYSTEM POSSIBLE AAAHAHAHAHAHAAHAAAAAAA

void ModularControl::activateMacro(ModularComponent modc, bool isRed) {
    if (modc == ModularComponent::MATCHLOAD) {
        // get nearest matchload on team
        //  goto it
        // TODO LATER
    }
}

std::vector<ModularControl::cord> ModularControl::getAvail(cord botLocation,
                                                           bool isRed) {
    return std::invoke(
        [](std::vector<teamCord> matchload,
           bool isRed) -> std::vector<ModularControl::cord> {
            std::vector<ModularControl::cord> r;
            for (int i = 0; i < matchload.size(); i++) {
                if (matchload[i].isRed == isRed) {
                    r.emplace_back(matchload[i].cordnate);
                }
            }
            return r;
        },
        matchload,
        isRed);
}
