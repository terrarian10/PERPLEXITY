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
#include "airCylinder.hpp"
#include "color_sort.hpp"
#include "consts.hpp"
#include "modularSubsystem.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/motor_group.hpp"
#include "pros/rtos.hpp"
#include <string>
#include <vector>

struct single_control {
    int motorID;
    double moveMPL;
};
struct multi_control {
    std::vector<single_control> soloCont;
    Outt_States id;
};
struct mecha_control {
    std::vector<multi_control> motorHandling;
    const char* taskID;
};

// Setup class
class Outtake : public subsystem {
  public:
    // different Outt_States for the outtake
    /**
     * @brief Code for an outtake object
     * @param outtake_mtrs The motors that control the outtake
     * @param state The state the outtake is in
     * @param RUNNING_VOLTAGE how much voltage to run the intake on
     */
    // Motorgroup is not motorgrouping
    // Should probably make it work but single motor 3 times ig

    Outtake(std::vector<pros::Motor>& motors,
            std::vector<AirCylinder>& air,
            ColourDetector colorDetector,
            mecha_control& mechHandler,
            Outt_States state,
            const std::uint32_t RUNNING_VOLTAGE = 12000)
        : motors(motors)
        , air(air)
        , colorDetector(colorDetector)
        , mechHandler(mechHandler)
        , state(state)
        , task(pros::Task([]() {}, mechHandler.taskID)) {
        move(state);
        cfg().baseSpeed = RUNNING_VOLTAGE;
    };

    // Initialize various functions and variables
    void move(int state) { this->state = Outt_States(state); };
    void emergency(int delay, std::vector<single_control> override);
    std::string log() override;

    void run() override;
    inline void suspend() override { task.suspend(); };
    void initialize() override;
    // inline void quit() override { task.remove(); };
    inline void suspend_time(int time) override {
        task.suspend();
        pros::delay(time);
        task.resume();
    };
    inline void resume() override { task.resume(); };
    inline void run_at_state(int state) {
        move(state);
        run();
    }
    int get_state() const { return state; }
    Outt_States get_state_enum() const { return state; }

  private:
    // Initialize various private variables
    pros::Task task;
    Outt_States state;
    ColourDetector colorDetector;
    std::vector<pros::Motor> motors;
    std::vector<AirCylinder> air;
    void loop();
    mecha_control& mechHandler;
};