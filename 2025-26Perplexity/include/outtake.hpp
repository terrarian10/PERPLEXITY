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

class outtake : public pros::MotorGroup {
  public:
    enum class State { OFF, HOARD, MIDDLE, TOP };

    /**
     * @brief Code for an outtake object
     * @param motors The motors that control the outtake
     */
    outtake(pros::AbstractMotor& outtake_mtrs,
            State state = State::OFF,
            std::uint32_t RUNNING_VOLTAGE = 12000)
        : pros::MotorGroup(outtake_mtrs)
        , state(state)
        , RUNNING_VOLTAGE(RUNNING_VOLTAGE)
        , task(pros::Task([]() {}, "outtake")) {
        move(state);
    };

    void move(State state);

    const State get_state() const { return state; }

  private:
    pros::Task task;

    const std::uint32_t RUNNING_VOLTAGE;

    State state;

    void loop(State state);
    // void change(State state);
};