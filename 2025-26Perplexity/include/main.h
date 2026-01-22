/**
 * \file main.h
 *
 * Contains common definitions and header files used throughout your PROS
 * project.
 *
 * Copyright (c) 2017-2021, Purdue University ACM SIGBots.
 * All rights reserved.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

// Import many thingies
#ifndef _PROS_MAIN_H_
#define _PROS_MAIN_H_

#include "airCylinder.hpp"
#define PROS_USE_SIMPLE_NAMES
#include "Outtake.hpp"
// #include "controller_data.hpp"

#include "api.h" // IWYU pragma: export
// #include "autons.hpp"
#include "lemlib/api.hpp" // IWYU pragma: export
#include "okapi/api.hpp"  // IWYU pragma: export
#include "pros/abstract_motor.hpp"
#include "pros/misc.h"
#include "pros/motor_group.hpp"

#ifdef __cplusplus
/**
 * You can add C++-only headers here
 */
#include "controller_data.hpp"

#include "Outtake.hpp"     // IWYU pragma: export
#include "airCylinder.hpp" // IWYU pragma: export
#include "autons.hpp"      // IWYU pragma: export
#include "zvirtual/virtualController.hpp"
#include <iostream> // IWYU pragma: export

extern pros::Controller master;
extern lemlib::Chassis chassis;
extern Outtake outtake;
extern AirCylinder scraper;
extern AirCylinder middle_scorer;
extern AirCylinder descorer_l;
extern virtualController temp;
extern mecha_control outtake_ctrl;
#endif

/**
 * Prototypes for the competition control tasks are redefined here to ensure
 * that they can be called from user code (i.e. calling autonomous from a
 * button press in opcontrol() for testing purposes).
 */

#ifdef __cplusplus
extern "C" {
extern pros::Gps gps;
#endif
void autonomous(void);
void initialize(void);
void initialize_macros();
void disabled(void);
void competition_initialize(void);
void opcontrol(void);
#ifdef __cplusplus
}
#endif

#endif // _PROS_MAIN_H_
