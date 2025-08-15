#include "Outtake.hpp"
#include "pros/rtos.h"
#include "pros/rtos.hpp"
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <vector>

void Outtake::move(std::string state) {
    std::cout << "Tasking" << std::endl;
    std::cout << "Tasking" << std::endl;
    std::cout << "Tasking" << std::endl;
    std::cout << "Tasking" << std::endl;

    this->state = state;
    // if (task.get_count() > 0) task.remove();
    std::cout << "Tasking" << std::endl;
    // please dont explode I barely understand how this works
    task.remove();
    std::cout << "Making Task" << std::endl;
    // 50/50 chance the task actually works
    task = pros::Task([this, state]() -> void { this->loop(state); },
                      mechHandler.taskID);
};

void Outtake::loop(std::string state) {
    uint32_t timer = 0;
    // Move motors differently depending on what needs to be done
    while (true) {

        for (auto& r : mechHandler.motorHandling) {

            if (state == r.id) {
                // std::cout << r.id << "\n";

                for (const auto& i : r.soloCont) {

                    this->motors.at(i.motorID).move_velocity(i.moveMPL * 12000);
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
        //     //         this->outtake_2.move_voltage(-1 * RUNNING_VOLTAGE);
        //     //         this->outtake_3.move_voltage(-1 * RUNNING_VOLTAGE);
        //     //         break;
        //     //     case State::MIDDLE:
        //     //         this->outtake_1.move_voltage(-1 * RUNNING_VOLTAGE);
        //     //         this->outtake_2.move_voltage(1 * RUNNING_VOLTAGE);
        //     //         this->outtake_3.move_voltage(-1 * RUNNING_VOLTAGE);
        //     //         break;
        //     //     case State::HOARD:
        //     //         this->outtake_1.move_voltage(-1 * RUNNING_VOLTAGE);
        //     //         this->outtake_2.move_voltage(-1 * RUNNING_VOLTAGE);
        //     //         this->outtake_3.move_voltage(1 * RUNNING_VOLTAGE);
        //     //         break;
        //     //     case State::BOTTOM:
        //     //         this->outtake_1.move_voltage(1 * RUNNING_VOLTAGE);
        //     //         this->outtake_2.move_voltage(1 * RUNNING_VOLTAGE);
        //     //         this->outtake_3.move_voltage(-1 * RUNNING_VOLTAGE);
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