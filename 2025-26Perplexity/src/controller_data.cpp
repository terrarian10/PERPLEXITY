#include "controller_data.hpp"
#include "Outtake.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include <iterator>

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

int getNearest(int** matrix, int start, int end) {}
