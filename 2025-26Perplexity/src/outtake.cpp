#include "outtake.hpp"
#include <cstdint>

void outtake::move(State state) {
    this->state = state;

    task.remove();

    task =
        pros::Task([this, state]() -> void { this->loop(state); }, "outtake");
};

void outtake::loop(State state) {
    uint32_t timer = 0;

    while (true) {
        switch (state) {
            case State::TOP:
                this[0].move_voltage(1 * RUNNING_VOLTAGE);
                this[1].move_voltage(1 * RUNNING_VOLTAGE);
                this[2].move_voltage(1 * RUNNING_VOLTAGE);
                break;
            case State::MIDDLE:
                this[0].move_voltage(1 * RUNNING_VOLTAGE);
                this[1].move_voltage(1 * RUNNING_VOLTAGE);
                this[2].move_voltage(1 * RUNNING_VOLTAGE);
                break;
            case State::HOARD:
                this[0].move_voltage(1 * RUNNING_VOLTAGE);
                this[1].move_voltage(1 * RUNNING_VOLTAGE);
                this[2].move_voltage(1 * RUNNING_VOLTAGE);
                break;
            case State::OFF:
                this[0].move_voltage(0 * RUNNING_VOLTAGE);
                this[1].move_voltage(0 * RUNNING_VOLTAGE);
                this[2].move_voltage(0 * RUNNING_VOLTAGE);
                break;
        }
        pros::delay(20);
    }
};