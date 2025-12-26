#include "Outtake.hpp"
#include "consts.hpp"
#include "pros/misc.h"
#include "pros/rtos.h"
#include "pros/rtos.hpp"
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

/*
How to use Outtake
move() loads the task
run() runs the current task



*/

void Outtake::run() { isRunning = true; };
void Outtake::halt() { isRunning = false; };

void Outtake::loop() {
    uint32_t timer = 0;
    // Move motors differently depending on what needs to be done

    if (isRunning) {
        for (auto& r : mechHandler.motorHandling) {
            if (state == r.control) {
                for (const auto& i : r.soloCont) {
                    if (i.motorID >= 0) {
                        this->motors.at(i.motorID).move_voltage(
                            i.moveMPL *
                            (this->motors.at(i.motorID).get_efficiency() > 0.9
                                 ? 12000
                                 : (i.efficiencyOverride * 12000)));
                    } else {
                        if (i.moveMPL != -1 && air.at(std::abs(i.motorID) - 1)
                                                       .get_value() != i.moveMPL
                                ? true
                                : false) {
                            this->air.at(std::abs(i.motorID) - 1)
                                .set_value(i.moveMPL == 1 ? true : false);
                        }
                    }
                }
            }
        }
    } else {
        for (const auto& i : idle_state) {
            if (i.motorID >= 0) {
                this->motors.at(i.motorID).move_voltage(i.moveMPL * 12000);
            } else {
                if (i.moveMPL != -1 &&
                            air.at(std::abs(i.motorID) - 1).get_value() !=
                                i.moveMPL
                        ? true
                        : false) {
                    this->air.at(std::abs(i.motorID) - 1)
                        .set_value(i.moveMPL == 1 ? true : false);
                }
            }
        }

        pros::delay(20);
    }
    pros::delay(20);
};

void Outtake::emergency(int delay, std::vector<single_control> override) {
    // task.suspend();
    // for (const auto& i : override) {
    //     if (i.motorID >= 0) {
    //         this->motors.at(i.motorID).move_voltage(i.moveMPL * 12000);
    //     } else {
    //         if (i.moveMPL != -1 &&
    //                     air.at(std::abs(i.motorID) - 1).get_value() !=
    //                     i.moveMPL
    //                 ? true
    //                 : false) {
    //             this->air.at(std::abs(i.motorID) - 1)
    //                 .set_value(i.moveMPL == 1 ? true : false);
    //             std::cout << "UPDATING PNEU" << std::endl;
    //         }
    //         std::cout << "Try update pneu" << std::endl;
    //     }
    // }

    // pros::delay(delay); // TODO:: ACCOUNT FOR MOTOR TORQUE
    //                     // HIGHER TORQUE IS LESS SPEED, HOWEVER
    //                     // THAT IS IN TURN DEPENDING ON WATTAGE
    //                     // BECAUSE WHYYYYYY SO USE THAT TOO
    // task.resume();
}

void Outtake::initialize() {
    move(pros::E_CONTROLLER_DIGITAL_L1);
    isRunning = false;

    // task = pros::Task([this]() -> void { this->loop(); },
    //                   TASK_PRIORITY_MIN,
    //                   4096,
    //                   mechHandler.taskID);
}

std::string Outtake::log() {
    std::string temps;
    for (const auto& i : this->motors) {

        temps = temps + "MOTORPORT: " + std::to_string(i.get_port()) +
                " OVERTEMP: " + std::to_string(i.is_over_temp()) + "\n";
    }

    return temps;
}

mechData Outtake::getData() {
    std::vector<motorData> motors;
    for (auto& i : this->motors) {
        motors.emplace_back(
            motorData{ i.get_temperature(),
                       i.get_actual_velocity(),
                       i.get_actual_velocity() / i.get_target_velocity(),
                       i.get_torque() });
    }

    return mechData{
        motors,
        this->get_state_ctrl(),
    };
}