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

// Import necessary classes
#include "pros/abstract_motor.hpp"
#include "pros/motor_group.hpp"

// Setup class
class Outtake : public pros::MotorGroup {
  public:
    // different states for the outtake
    enum class State { OFF, HOARD, MIDDLE, TOP };

    /**
     * @brief Code for an outtake object
     * @param outtake_mtrs The motors that control the outtake
     * @param state The state the outtake is in
     * @param RUNNING_VOLTAGE how much voltage to run the intake on
     */
    Outtake(pros::AbstractMotor& outtake_mtrs,
            State state = State::OFF,
            std::uint32_t RUNNING_VOLTAGE = 12000)
        : pros::MotorGroup(outtake_mtrs)
        , state(state)
        , RUNNING_VOLTAGE(RUNNING_VOLTAGE)
        , task(pros::Task([]() {}, "outtake")) {
        move(state);
    };
    // Initialize various functions and variables
    void move(State state);

    const State get_state() const { return state; }

  private:
    // Initialize various private variables
    pros::Task task;

    const std::uint32_t RUNNING_VOLTAGE;

    State state;

    void loop(State state);
};