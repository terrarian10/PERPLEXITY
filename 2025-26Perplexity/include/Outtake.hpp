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
#include "color_sort.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/motor_group.hpp"
#include <string>
#include <vector>

// Setup class
class Outtake {
  public:
    // different states for the outtake
    /**
     * @brief Code for an outtake object
     * @param outtake_mtrs The motors that control the outtake
     * @param state The state the outtake is in
     * @param RUNNING_VOLTAGE how much voltage to run the intake on
     */
    // Motorgroup is not motorgrouping
    // Should probably make it work but single motor 3 times ig
    struct single_control {
        int motorID;
        int moveMPL;
    };
    struct multi_control {
        std::vector<single_control> soloCont;
        std::string id;
    };
    struct mecha_control {
        std::vector<multi_control> motorHandling;
        const char* taskID;
    };
    Outtake(std::vector<pros::Motor>& motors,
            ColourDetector colorDetector,
            mecha_control& mechHandler,
            std::string state,
            const std::uint32_t RUNNING_VOLTAGE = 12000)
        : motors(motors)
        , colorDetector(colorDetector)
        , mechHandler(mechHandler)
        , state(state)
        , RUNNING_VOLTAGE(RUNNING_VOLTAGE)
        , task(pros::Task([]() {}, mechHandler.taskID)) {
        move(state);
    };

    // Initialize various functions and variables
    void move(std::string state);
    void emergency(int delay, std::vector<single_control> override);

    std::string get_state() const { return state; }

  private:
    // Initialize various private variables
    pros::Task task;
    std::uint32_t RUNNING_VOLTAGE;
    std::string state;
    ColourDetector colorDetector;
    std::vector<pros::Motor> motors;
    void loop(std::string state);
    mecha_control& mechHandler;
};