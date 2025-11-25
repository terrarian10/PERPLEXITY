#include "tick.hpp"
#include "commandHandling.hpp"
#include "main.h"
Scheduler botScheduler();
void ticker::tick() {
    switch (bot_state) {
        case DISABLED: break;
        case OPCTRL: break;
        case AUTONCTRL: break;
    }
}