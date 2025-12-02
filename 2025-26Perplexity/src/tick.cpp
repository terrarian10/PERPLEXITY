#include "tick.hpp"
#include "autons.hpp"
#include "commandHandling.hpp"
#include "main.h"
#include "zcommands/chassisCommands.hpp"
Scheduler botScheduler();
void ticker::tick() {
    switch (bot_state) {
        case DISABLED: break;
        case OPCTRL: break;
        case AUTONCTRL: break;
    }
}
void cmd_opctrl() {
    virtualController temp{};
    botScheduler().enqueue<group_repeat_cmd>(tank_c(chassis, temp));
}