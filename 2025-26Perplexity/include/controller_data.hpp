/**
 * @file outtake.hpp
 * @author Gabriel Hein
 * @brief Outtake handler
 * @version 0.1
 * @date 2025-07-13
 *
 * @copyright Copyright (c) 2025
 *
 */

#pragma once

// Import unnecessary classes
#include "Outtake.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include <cmath>
#include <functional>
#include <optional>
#include <vector>
// Setup class
class ModularControl {
  public:
    // different states for the outtake
    /**
     * @brief Code for a info display on controller
     * @param chassis is drivetrain
     * @param conntroller is controller
     * @param outtake is outtake

     */
    // What even is this, why do I have it, why is this needed
    // Time to ruin the brains processing power because why not
    ModularControl(lemlib::Chassis& chassis,
                   pros::Controller& controller,
                   Outtake outtake,
                   int address = 1)
        : chassis(chassis)
        , controller(controller)
        , address(address)
        , outtake(outtake) {};
    // Point :)
    struct cord {

        double x;
        double y;
        double rot;

        std::optional<bool> isRed;
    };
    struct macro {
        std::vector<cord> cords;
        const char* representation;
    };
    // Initialize various functions and variables
    void updateDisplay(const macro& macro);
    // Im spiraling into insanity
    // There are thousands of better ways to do this, but everything is in 1
    // header file now and its amaingggg :'(
    // found some ways to fix some mem and cpu problems so we dont implode the
    // poor vex brain (thanks internet) BUT THERES DEFINETLY STILL MORE
    // Schitzophrenia final1 boss
    std::function<std::vector<cord>(const std::vector<cord>& matchload,
                                    bool isRed)>
        getAvail = [](const std::vector<cord>& matchload,
                      bool isRed) -> std::vector<ModularControl::cord> {
        std::vector<cord> r;
        r.reserve(matchload.size());
        for (const auto& i : matchload) {
            if (i.isRed == isRed || i.isRed == std::nullopt) { r.push_back(i); }
        }
        return r;
    };
    // THE GREATEST ONE LINER INITIALIZATION
    void activateMacro(macro mac, bool isRed);
    // Schitzophrenia {FIXED, NO LONGER SCHITZO}
    // Its like the (:PYTHAGOREAN THEOREM:)
    double getDistance(const cord& c1, const cord& c2) {
        return sqrt(pow((c1.x - c2.x), 2) + pow((c1.y - c2.y), 2));
    };

    // Wanna see me do it again?
    // Gets the nearest coordinate to a given coordinate
    std::function<cord(const cord&, const std::vector<cord>&)> getNearest =
        [&](const cord& botCord, const std::vector<cord>& locations) -> cord {
        cord r = locations.front();
        auto rd = getDistance(botCord, r);
        for (const auto& i : locations) {
            auto d = getDistance(botCord, i);
            if (d < rd) {
                r = i;
                rd = d;
            }
        }
        return r;
    };
    cord getClosest(cord botLocation, std::vector<cord> targets, bool isRed);
    void incrementAddress(int amount);
    const int get_active_address() const { return address; }

  private:
    lemlib::Chassis& chassis;
    Outtake outtake;
    int address;
    pros::Controller& controller;
    std::vector<cord> macros;

    // Matrix Coordinates YIPPEE
    // {X, Y, ROTATION, TEAM}
};