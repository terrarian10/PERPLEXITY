#include "autons.hpp"
#include "commandHandling.hpp"
#include "consts.hpp"
#include "lemlib/asset.hpp"
#include "lemlib/pose.hpp"
#include "logger.hpp"
#include "main.h"
#include "pros/motors.h"
#include "pros/rtos.h"
#include "pros/rtos.hpp"
#include <cmath>
#include <cstdint>
#include <functional>
#include <iostream>
#include <sstream>

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
inline float adjustHeading(double baseDeg, int mx, int my) {
    float h = baseDeg;

    // Mirror across Y-axis (left ↔ right)
    if (mx != -1) {
        h = std::fmod(360.0 - h, 360.0);
        if (h < 0) h += 360.0;
    }

    // Mirror across X-axis (top ↔ bottom)
    if (my != +1) {
        h = std::fmod(180.0 - h, 360.0);
        if (h < 0) h += 360.0;
    }

    return h;
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

void auton_one_side(float mx, float my, bool doSkills) {

    int iteration = pros::millis();
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    chassis.setPose(mx * 160_cm, my * 48.5_cm, adjustHeading(90, mx, my));
    // gps_initialize(mx,
    //                my,
    //                chassis.getPose().x,
    //                chassis.getPose().y,
    //                adjustHeading(90, mx, my),
    //                gps);
    // chassis.setPose(chassis.getPose().x, chassis.getPose().y, 90);
    //  Go to short goal
    // autonSchedule.enqueue<move_pose_c>(
    //     lemlib::Pose{ mx * 130_cm, my * float(48.5_cm), 90 }, chassis);
    autonSchedule.enqueue<move_point_c>(
        mx * 150_cm, my * 50.5_cm, chassis, poseCFG{ false, 120 });
    autonSchedule.enqueue<move_pose_c>(
        lemlib::Pose{ mx * 105_cm, my * 50_cm, adjustHeading(90, mx, my) },
        chassis,
        poseCFG{ false, 100, 127 });

    autonSchedule.enqueue<outtake_c>(Outt_States::HOARD, outtake);
    autonSchedule.enqueue<move_pose_c>(
        lemlib::Pose{ mx * 50_cm, my * (50_cm), adjustHeading(135, mx, my) },
        chassis,
        poseCFG{ false, 20, 80 });
    autonSchedule.enqueue<move_pose_c>(
        lemlib::Pose{ mx * 60_cm, my * 60_cm, adjustHeading(315, mx, my) },
        chassis,
        poseCFG{ .reversed = false });

    autonSchedule.enqueue<turn_heading_c>(adjustHeading(315, mx, my), chassis);
    autonSchedule.enqueue<move_pose_c>(
        lemlib::Pose{ mx * 40_cm, my * 40_cm, adjustHeading(315, mx, my) },
        chassis,
        poseCFG{ .reversed = true });
    autonSchedule.enqueue<move_pose_c>(
        lemlib::Pose{ mx * 40_cm, my * 40_cm, adjustHeading(315, mx, my) },
        chassis,
        poseCFG{ .reversed = true });
    autonSchedule.enqueue<outtake_c>(Outt_States::MIDDLE, outtake);
    autonSchedule.enqueue<wait_c>(1000);

    autonSchedule.enqueue<outtake_c>(Outt_States::OFF, outtake);
    // autonSchedule.enqueue<move_pose_c>(lemlib::Pose{ -44_cm, 40_cm, 135 },
    //                                    chassis);
    // autonSchedule.enqueue<move_point_c>(
    //     mx * 60_cm, my * 60_cm, chassis, poseCFG{ true });
    // autonSchedule.enqueue<move_pose_c>(
    //     lemlib::Pose{ mx * 60_cm, my * 60_cm, 135 }, chassis, poseCFG{ true
    //     });

    // autonSchedule.enqueue<outtake_c>(Outt_States::OFF, outtake);

    // autonSchedule.enqueue<wait_c>(200);

    while (autonSchedule.tick()) {
        pros::delay(10);
        // std::cout << chassis.getPose().x << std::endl;
    }
    std::cout << double(pros::millis() - iteration) / 1000 << std::endl;
    std::cout << chassis.getPose().x * 2.54 << std::endl;
    std::cout << chassis.getPose().y * 2.54 << std::endl;
}

void testing_pid() {
    chassis.setPose(0, 0, 0);
    chassis.moveToPose(0, 100, 0, 6000);
}
