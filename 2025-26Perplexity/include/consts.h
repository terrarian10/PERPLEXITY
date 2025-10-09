#pragma once

enum Outt_States {
    OFF = 0,
    TOP = 1,
    BOTTOM = 2,
    MIDDLE = 3,
    BOTTOM_STORE = 4,
    TOP_STORE = 5,
    UNJAM = 6,
    UNJAM_NO_RELEASE = 7
};

struct poseCFG {
    bool reversed = false;
    int minSpeed = 40;
    int maxSpeed = 120;
    int timeout = 5000;
    float earlyExitRange = 0.2;
};