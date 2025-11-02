#pragma once

enum Outt_States { OFF = 0, TOP = 1, BOTTOM = 2, MIDDLE = 3 };

struct poseCFG {
    bool reversed = false;
    int minSpeed = 40;
    int maxSpeed = 120;
    int timeout = 5000;
    float earlyExitRange = 0.2;
};