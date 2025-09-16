#include "autons.hpp"
#include "commandHandling.hpp"
#include "lemlib/asset.hpp"
#include "main.h"
#include "pros/rtos.h"
#include "pros/rtos.hpp"
#include <cmath>
#include <cstdint>
#include <functional>
#include <iostream>

// These are out of 127
const int DRIVE_SPEED = 110;
const int TURN_SPEED = 90;
const int SWING_SPEED = 90;
Scheduler autonSchedule;
ASSET(auton_test_txt);
void absTurn(double angle, bool blocking = true, int timeout = 750) {
    // std::cout << "theta:" << operator""_deg(angle)
    // << " new x:" << 1e5 * std::sin(operator""_deg(angle))
    // << " new y:" << 1e5 * std::cos(operator""_deg(angle)) << '\n';
    chassis.turnToHeading(angle, timeout);

    if (blocking) chassis.waitUntilDone();
}

// Move relative to current location
void relativeMove(double distance, bool blocking = true, int timeout = 4000) {
    std::cerr << "in function\v";
    lemlib::Pose pose = chassis.getPose(true);

    std::cerr << "moving from x:" << pose.x << " y:" << pose.y
              << " theta:" << pose.theta << '\v'
              << "Moving to x:" << pose.x + distance * std::sin(pose.theta)
              << " y:" << pose.y + distance * std::cos(pose.theta) << '\n';

    chassis.moveToPoint(pose.x + distance * std::sin(pose.theta),
                        pose.y + distance * std::cos(pose.theta),
                        timeout,
                        { distance > 0 });
    if (blocking) chassis.waitUntilDone();
}

// ///
// // Constants
// ///
void default_constants() {}

// Skills auton
void auton_skills() {
    uint32_t start_time = pros::millis();

    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(0, 20, 100000);
    // Outputs total time, for testing
    std::cout << "\033[7mTIME time: " << (pros::millis() - start_time) / 1000.0
              << " sec\033[0m\n";
}

// Individial side autons
void autons_positive_red() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    chassis.setPose(-155_cm, -45_cm, 90);
    chassis.turnToHeading(180, 100000);

    // autonSchedule.enqueue<move_pose_c>(lemlib::Pose{ -45_cm, -60_cm, 90
    // },
    //                                    chassis);
    // autonSchedule.enqueue<move_pose_c>(lemlib::Pose{ -17, -17, 225 },
    // chassis); autonSchedule.enqueue<move_pose_c>(lemlib::Pose{ -36, -31,
    // 225
    // }, chassis);

    // autonSchedule.enqueue<move_pose_c>(lemlib::Pose{ -56, -47, 270 },
    // chassis); autonSchedule.enqueue<move_pose_c>(
    //     lemlib::Pose{ -86_cm, -121_cm, 270 }, chassis, true);

    // chassis.moveToPose(-100_cm, -45_cm, 90, 5000);

    // chassis.moveToPose(-45_cm, -60_cm, 90, 5000);
    // chassis.moveToPose(-30_cm, -45_cm, 225, 5000);
    // chassis.moveToPose(-140_cm, -120_cm, 270, 5000);
    // chassis.moveToPose(-90_cm, -120_cm, 270, 5000);

    // chassis.turnToHeading(180, 5000);
    // chassis.waitUntilDone();
    // chassis.turnToHeading(360, 5000);

    // while (true) {
    //     autonSchedule.tick();
    //     pros::delay(10);
    // }
}

void autons_positive_blue() {}

void autons_negative_red() {}

void autons_negative_blue() {}
