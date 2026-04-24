#pragma once
#include "commandHandling.hpp"
#include "pros/motor_group.hpp"

enum botState { AUTONCTRL, OPCTRL, DISABLED };

class ticker {
  public:
    Scheduler scheduler{}; // persistent scheduler
    Scheduler& botScheduler() { return scheduler; }

    virtualController mainVirutal{};
    void tick();
    void command_opcontrol(pros::MotorGroup& left, pros::MotorGroup& right);
};
