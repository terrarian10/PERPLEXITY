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
#include "pros/device.hpp"
#include "pros/misc.h"
#include "pros/motor_group.hpp"
#include "pros/rtos.hpp"
#include <string>
#include <vector>
struct motorData {
    double overheatLevel;
    double velocity;
    double friction;
    double torque;
};

struct single_control {
    int motorID;
    double moveMPL;
};
struct multi_control {
    std::vector<single_control> soloCont;
    pros::controller_digital_e_t control;
};
struct mecha_control {
    std::vector<multi_control> motorHandling;
    const char* taskID;
};
struct mechData {
    std::vector<motorData> motors_info;
    pros::controller_digital_e_t control;
    multi_control current_direct_control;
    pros::Task* current_task;
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
            std::vector<single_control>& idle_state,
            const std::uint32_t RUNNING_VOLTAGE = 12000)
        : motors(motors)
        , air(air)
        , colorDetector(colorDetector)
        , mechHandler(mechHandler)
        , idle_state(idle_state)
        , state(pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_L1)
        , isRunning(false)
        , task(pros::Task([]() {}, mechHandler.taskID)) {
        move(state);
        cfg().baseSpeed = RUNNING_VOLTAGE;
    };

    // Initialize various functions and variables
    void move(pros::controller_digital_e_t state) { this->state = state; };
    void emergency(int delay, std::vector<single_control> override);
    std::string log() override;
    void halt();
    void run() override;
    mechData getData();
    bool getRunning() { return isRunning; }
    inline void suspend() override { task.suspend(); };
    void initialize() override;
    // inline void quit() override { task.remove(); };
    inline void suspend_time(int time) override {
        task.suspend();
        pros::delay(time);
        task.resume();
    };
    inline void resume() override { task.resume(); };
    inline void run_at_state(pros::controller_digital_e_t state) {
        move(state);
        run();
    }
    void loop();
    int get_state() const { return state; }
    pros::controller_digital_e_t get_state_ctrl() const { return state; }

  private:
    // Initialize various private variables
    pros::Task task;
    pros::controller_digital_e_t state;
    ColourDetector colorDetector;
    std::vector<pros::Motor> motors;
    std::vector<single_control>& idle_state;
    std::vector<AirCylinder> air;
    bool isRunning;

    mecha_control& mechHandler;
};