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
#include "pros/abstract_motor.hpp"
#include "pros/motor_group.hpp"

// Setup class
class Outtake {
  public:
    // different states for the outtake
    enum class State { OFF, HOARD, MIDDLE, TOP, BOTTOM };
    /**
     * @brief Code for an outtake object
     * @param outtake_mtrs The motors that control the outtake
     * @param state The state the outtake is in
     * @param RUNNING_VOLTAGE how much voltage to run the intake on
     */
    // Motorgroup is not motorgrouping
    // Should probably make it work but single motor 3 times ig
    Outtake(pros::Motor outtake_1,
            pros::Motor outtake_2,
            pros::Motor outtake_3,
            State state = State::OFF,
            const std::uint32_t RUNNING_VOLTAGE = 12000)
        : outtake_1(outtake_1)
        , outtake_2(outtake_2)
        , outtake_3(outtake_3)
        , RUNNING_VOLTAGE(RUNNING_VOLTAGE)
        , task(pros::Task([]() {}, "Outtake")) {
        move(state);
    };

    // Initialize various functions and variables
    void move(State state);

    const State get_state() const { return state; }

  private:
    // Initialize various private variables
    pros::Task task;
    std::uint32_t RUNNING_VOLTAGE;
    State state;
    pros::Motor outtake_1;
    pros::Motor outtake_2;
    pros::Motor outtake_3;
    void loop(State state);
};