#include "main.h"
#include "pros/gps.h"
#include "pros/gps.hpp"
#include "pros/rtos.hpp"
#include <array>
#include <cmath>

void gps_initialize(int _mx,
                    int _my,
                    double startX,
                    double startY,
                    double _heading,
                    pros::Gps& _gps);
void gpsRefinerTask(pros::Gps& gps);