#include "main.h"
enum botState { AUTONCTRL, OPCTRL, DISABLED };

class ticker {
  public:
    void tick();
    botState bot_state;
};
