#include "controller_data.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "pros/misc.hpp"
#include <functional>
#include <vector>

void ModularControl::updateDisplay(const macro& macro) {
    // std::cout << macro.representation << "\n\n";
    controller.print(0, 0, macro.representation);
}

// I HAVE DECIDED ON THE MOST STUPID CONVOLUTED OVERPOWERED OVERENGINEERED
// MACRO SYSTEM POSSIBLE AAAHAHAHAHAHAAHAAAAAAA

void ModularControl::activateMacro(macro mac, bool isRed) {
    chassis.cancelAllMotions();
    cord closest = getClosest(
        { double(chassis.getPose().x), double(chassis.getPose().y), 0 },
        mac.cords,
        isRed);
    chassis.moveToPose(
        float(closest.x), float(closest.y), float(closest.rot), 5000);
}

// std::vector<ModularControl::cord> ModularControl::getAvail(cord
// botLocation,
//                                                            bool isRed) {
//     return std::invoke(
//         [](std::vector<teamCord> matchload,
//            bool isRed) -> std::vector<ModularControl::cord> {
//             std::vector<ModularControl::cord> r;
//             for (int i = 0; i < matchload.size(); i++) {
//                 if (matchload[i].isRed == isRed) {
//                     r.emplace_back(matchload[i].cordnate);
//                 }
//             }
//             return r;
//         },
//         matchload,
//         isRed);
// }

ModularControl::cord ModularControl::getClosest(cord botLocation,
                                                std::vector<cord> targets,
                                                bool isRed) {
    return getNearest(botLocation, getAvail(targets, isRed));
}

void ModularControl::incrementAddress(int amount) { address += amount; }