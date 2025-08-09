#include "Outtake.hpp"
#include "pros/rtos.h"
#include "pros/rtos.hpp"
#include <cstddef>
#include <cstdint>

void Outtake::move(State state) {
    this->state = state;
    // if (task.get_count() > 0) task.remove();
    std::cout << "Tasking" << std::endl;
    // please dont explode I barely understand how this works
    task.remove();
    std::cout << "Making Task" << std::endl;
    // 50/50 chance the task actually works
    task =
        pros::Task([this, state]() -> void { this->loop(state); }, "Outtake");
};

void Outtake::loop(State state) {
    uint32_t timer = 0;
    // Move motors differently depending on what needs to be done
    while (true) {

        switch (state) {
            // Guess and check final boss phase 2
            case State::TOP:
                // Wants ~pointers~ for some reason
                this->outtake_1.move_voltage(-1 * RUNNING_VOLTAGE);
                this->outtake_2.move_voltage(-1 * RUNNING_VOLTAGE);
                this->outtake_3.move_voltage(-1 * RUNNING_VOLTAGE);
                break;
            case State::MIDDLE:
                this->outtake_1.move_voltage(-1 * RUNNING_VOLTAGE);
                this->outtake_2.move_voltage(1 * RUNNING_VOLTAGE);
                this->outtake_3.move_voltage(-1 * RUNNING_VOLTAGE);
                break;
            case State::HOARD:
                this->outtake_1.move_voltage(-1 * RUNNING_VOLTAGE);
                this->outtake_2.move_voltage(-1 * RUNNING_VOLTAGE);
                this->outtake_3.move_voltage(1 * RUNNING_VOLTAGE);
                break;
            case State::BOTTOM:
                this->outtake_1.move_voltage(1 * RUNNING_VOLTAGE);
                this->outtake_2.move_voltage(1 * RUNNING_VOLTAGE);
                this->outtake_3.move_voltage(-1 * RUNNING_VOLTAGE);
                break;
            case State::OFF:

                this->outtake_1.move_voltage(0);
                this->outtake_2.move_voltage(0);
                this->outtake_3.move_voltage(0);
                break;
        }
        pros::delay(20);
    }
};

void Outtake::ejection() {
    task.suspend();
    this->outtake_1.move_voltage(-1 * RUNNING_VOLTAGE);
    this->outtake_2.move_voltage(1 * RUNNING_VOLTAGE);
    this->outtake_3.move_voltage(1 * RUNNING_VOLTAGE);
    pros::delay(350);
    task.resume();
}