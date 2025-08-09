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

// Setup class
class ModularControl {
  public:
    // BLAFN (Big long annoying function name)
    enum class ModularComponent { MATCHLOAD, SHORTGOAL, LONGGOAL };

    // different states for the outtake
    /**
     * @brief Code for a info display on controller
     * @param chassis is drivetrain
     * @param conntroller is controller
     * @param outtake is outtake

     */
    // What even is this, why do I have it, why is this needed
    // Time to ruin the brains processing power because why not
    ModularControl(lemlib::Chassis chassis,
                   pros::Controller controller,
                   Outtake outtake,
                   ModularComponent modc = ModularComponent::MATCHLOAD)
        : chassis(chassis)
        , controller(controller)
        , outtake(outtake) {
        updateDisplay(modc);
    };

    // Initialize various functions and variables
    void updateDisplay(ModularComponent modc);
    void activateMacro(ModularComponent modc, bool isRed);
    // Schitzophrenia
    std::function<int(int x1, int y1, int x2, int y2)> getDistance =
        [](int x1, int y1, int x2, int y2) -> int {
        return std::hypot((x1 - x2), (y1 - y2));
    };

    const ModularComponent get_active_component() const { return modc; }

  private:
    lemlib::Chassis& chassis;
    Outtake outtake;

    pros::Controller controller;
    ModularComponent modc;
    // Matrix Coordinates YIPPEE
    // {X, Y, ROTATION, TEAM}
    int matchload_matrix[4][4] = { { 0, 0, 0, 0 },
                                   { 0, 0, 0, 0 },
                                   { 0, 0, 0, 0 },
                                   { 0, 0, 0, 0 } };
    int gamefield_matrix[2][2] = { { -200, -200 }, { 200, 200 } };
};