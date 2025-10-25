#include "Outtake.hpp"
#include "consts.h"
#include "pros/rtos.h"
#include "pros/rtos.hpp"
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <string>
#include <vector>

/*
How to use Outtake
move() loads the task
run() runs the current task



*/

void Outtake::run() {

    // please dont explode I barely understand how this works
    // if (task.get_state() != pros::E_TASK_STATE_DELETED) { task.remove(); }

    // 50/50 chance the task actually works
    if (task.get_state() == pros::E_TASK_STATE_INVALID ||
        task.get_state() == pros::E_TASK_STATE_DELETED) {}
};

void Outtake::loop() {
    uint32_t timer = 0;
    // Move motors differently depending on what needs to be done
    while (true) {

        for (auto& r : mechHandler.motorHandling) {

            if (state == r.id) {
                // std::cout << r.id << "\n";
                // std::cout << std::endl;

                for (const auto& i : r.soloCont) {
                    if (i.motorID >= 0) {
                        this->motors.at(i.motorID).move_voltage(i.moveMPL *
                                                                12000);
                    } else {
                        this->air.at(std::abs(i.motorID) - 1)
                            .set_value(i.moveMPL == 1 ? true : false);
                    }
                }
            }
        }
        pros::delay(20);

        // switch (state) {
        //     // Guess and check final boss phase 2
        //     //     case State::TOP:
        //     //         // Wants ~pointers~ for some reason
        //     //         this->motors.at(0).mo->outtake_1.move_voltage(-1 *
        //     // RUNNING_VOLTAGE);
        //     //         this->outtake_2.move_voltage(-1 *
        //     RUNNING_VOLTAGE);
        //     //         this->outtake_3.move_voltage(-1 *
        //     RUNNING_VOLTAGE);
        //     //         break;
        //     //     case State::MIDDLE:
        //     //         this->outtake_1.move_voltage(-1 *
        //     RUNNING_VOLTAGE);
        //     //         this->outtake_2.move_voltage(1 * RUNNING_VOLTAGE);
        //     //         this->outtake_3.move_voltage(-1 *
        //     RUNNING_VOLTAGE);
        //     //         break;
        //     //     case State::HOARD:
        //     //         this->outtake_1.move_voltage(-1 *
        //     RUNNING_VOLTAGE);
        //     //         this->outtake_2.move_voltage(-1 *
        //     RUNNING_VOLTAGE);
        //     //         this->outtake_3.move_voltage(1 * RUNNING_VOLTAGE);
        //     //         break;
        //     //     case State::BOTTOM:
        //     //         this->outtake_1.move_voltage(1 * RUNNING_VOLTAGE);
        //     //         this->outtake_2.move_voltage(1 * RUNNING_VOLTAGE);
        //     //         this->outtake_3.move_voltage(-1 *
        //     RUNNING_VOLTAGE);
        //     //         break;
        //     //     case State::OFF:

        //     //         this->outtake_1.move_voltage(0);
        //     //         this->outtake_2.move_voltage(0);
        //     //         this->outtake_3.move_voltage(0);
        //     //         break;
        //     // }
    }
    pros::delay(20);
};

void Outtake::emergency(int delay, std::vector<single_control> override) {
    task.suspend();
    for (const auto& r : override) {
        this->motors.at(r.motorID).move_velocity(r.moveMPL * 12000);
    }

    pros::delay(delay); // TODO:: ACCOUNT FOR MOTOR TORQUE
                        // HIGHER TORQUE IS LESS SPEED, HOWEVER
                        // THAT IS IN TURN DEPENDING ON WATTAGE
                        // BECAUSE WHYYYYYY SO USE THAT TOO
    task.resume();
}

void Outtake::initialize() {
    move(Outt_States::OFF);
    task = pros::Task([this]() -> void { this->loop(); },
                      TASK_PRIORITY_MIN,
                      4096,
                      mechHandler.taskID);
}

std::string Outtake::log() {
    std::string temps;
    for (const auto& i : this->motors) {

        temps = temps + "MOTORPORT: " + std::to_string(i.get_port()) +
                " OVERTEMP: " + std::to_string(i.is_over_temp()) + "\n";
    }

    return temps;
}