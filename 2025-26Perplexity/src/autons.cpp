#include "autons.hpp"
#include "commandHandling.hpp"
#include "consts.hpp"
#include "lemlib/asset.hpp"
#include "lemlib/pose.hpp"
#include "main.h"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rtos.h"
#include "pros/rtos.hpp"
#include "zcommands/chassisCommands.hpp"
#include "zcommands/mechCommands.hpp"
#include <cmath>
#include <cstdint>
#include <functional>
#include <iostream>
#include <sstream>
#include <unordered_map>
#define OUT_TOP pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_R1
#define OUT_BOTTOM pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_L2
#define OUT_MIDDLE pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_R2
#define OUT_HOARD pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_L1

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
    bool doDescore = !doSkills;
    int iteration = pros::millis();
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    chassis.setPose(mx * 124_cm, my * 52_cm, adjustHeading(90, mx, my));
    autonSchedule.enqueue<setPose_c>(
        lemlib::Pose{ mx * 124_cm, my * 50_cm, adjustHeading(90, mx, my) });

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
    // autonSchedule.enqueue<move_point_c>(
    //     mx * 150_cm, my * 50.5_cm, chassis, poseCFG{ false, 120 });
    // autonSchedule.enqueue<move_pose_c>(
    //     lemlib::Pose{ mx * 105_cm, my * 50_cm, adjustHeading(90, mx, my) },
    //     chassis,
    //     poseCFG{ false, 100, 127 });
    autonSchedule.enqueue<move_pose_c>(
        lemlib::Pose{ mx * 90_cm, my * (62_cm), adjustHeading(90, mx, my) },
        chassis,
        poseCFG{ false, 80, 127 });
    autonSchedule.enqueue<mech_state_c>(OUT_HOARD, outtake);
    autonSchedule.enqueue<move_pose_c>(
        lemlib::Pose{ mx * 40_cm, my * (65_cm), adjustHeading(90, mx, my) },
        chassis,
        poseCFG{ false, 50, 50 });
    autonSchedule.enqueue<move_pose_c>(
        lemlib::Pose{ mx * 50_cm, my * 59_cm, adjustHeading(315, mx, my) },
        chassis,
        poseCFG{ .reversed = false, .minSpeed = 100, .maxSpeed = 120 });

    // autonSchedule.enqueue<turn_heading_c>(adjustHeading(315, mx, my),
    // chassis);
    std::cout << adjustHeading(315, mx, my);
    // autonSchedule.enqueue<togglePneu_c>(middle_scorer);

    autonSchedule.enqueue<move_pose_c>(
        lemlib::Pose{ mx * 41_cm, my * 43_cm, adjustHeading(315, mx, my) },
        chassis,
        poseCFG{ .reversed = true,
                 .minSpeed = 100,
                 .maxSpeed = 127,
                 .earlyExitRange = 3 });

    autonSchedule.enqueue<move_pose_c>(
        lemlib::Pose{ mx * 37_cm, my * 37_cm, adjustHeading(315, mx, my) },
        chassis,
        poseCFG{ .reversed = true, .minSpeed = 20, .timeout = 700 });
    autonSchedule.enqueue<move_pose_c>(
        lemlib::Pose{ mx * 0_cm, my * 0_cm, adjustHeading(315, mx, my) },
        chassis,
        poseCFG{ .reversed = true, .minSpeed = 20, .timeout = 120 });
    // autonSchedule.enqueue<move_pose_c>(
    //     lemlib::Pose{ mx * 40_cm, my * 40_cm, adjustHeading(315, mx, my) },
    //     chassis,
    //     poseCFG{ .reversed = true }); a^2+a^2=28.875
    if (doSkills) {
        autonSchedule.enqueue<mech_state_c>(OUT_BOTTOM, outtake);
        autonSchedule.enqueue<move_pose_c>(
            lemlib::Pose{ mx * 0_cm, my * 0_cm, adjustHeading(315, mx, my) },
            chassis,
            poseCFG{ .reversed = true,
                     .minSpeed = 30,
                     .maxSpeed = 40,
                     .timeout = doSkills ? 125 : 0 });
    }
    autonSchedule.enqueue<mech_state_c>(OUT_MIDDLE, outtake);
    autonSchedule.enqueue<move_pose_c>(
        lemlib::Pose{ mx * 0_cm, my * 0_cm, adjustHeading(315, mx, my) },
        chassis,
        poseCFG{ .reversed = true,
                 .minSpeed = 30,
                 .maxSpeed = 40,
                 .timeout = doSkills ? 4000 : 1500 });

    autonSchedule.enqueue<togglePneu_c>(descorer_l);
    if (doDescore) {

        autonSchedule.enqueue<togglePneu_c>(middle_scorer);
        autonSchedule.enqueue<move_pose_c>(
            lemlib::Pose{
                mx * 3000_cm, my * 3000_cm, adjustHeading(315, mx, my) },
            chassis,
            poseCFG{ .reversed = false,
                     .minSpeed = 60,
                     .maxSpeed = 70,
                     .timeout = 150 });
        autonSchedule.enqueue<move_pose_c>(
            lemlib::Pose{ mx * 0_cm, my * 0_cm, adjustHeading(315, mx, my) },
            chassis,
            poseCFG{ .reversed = true, .minSpeed = 80, .timeout = 300 });
    }
    // autonSchedule.enqueue<setPose_c>(
    //     lemlib::Pose{ mx * float(47.27_cm),
    //                   my * float(47.27_cm),
    //                   adjustHeading(315, mx, my) });

    autonSchedule.enqueue<move_point_c>(
        mx * 109_cm,
        my * 110_cm,
        chassis,
        poseCFG{ .reversed = false, .minSpeed = 100, .earlyExitRange = 1 });
    autonSchedule.enqueue<togglePneu_c>(scraper);

    // autonSchedule.enqueue<move_point_c>(
    //     mx * 109_cm,
    //     my * 121_cm,
    //     chassis,
    //     poseCFG{ .reversed = false, .minSpeed = 40 });
    autonSchedule.enqueue<outtake_toggle_c>(false, outtake);

    // autonSchedule.enqueue<turn_heading_c>(
    //     adjustHeading(270, mx, my), chassis, poseCFG{ .minSpeed = 40 });
    autonSchedule.enqueue<move_pose_c>(
        lemlib::Pose{ mx * 130_cm, my * 110_cm, adjustHeading(270, mx, my) },
        chassis,
        poseCFG{ .reversed = false, .minSpeed = 80 });
    autonSchedule.enqueue<move_pose_c>(
        lemlib::Pose{ mx * 400_cm, my * 110_cm, adjustHeading(270, mx, my) },
        chassis,
        poseCFG{ .reversed = false,
                 .minSpeed = 60,
                 .maxSpeed = 60,
                 .timeout = 1250 });
    if (!doDescore) { autonSchedule.enqueue<togglePneu_c>(middle_scorer); }

    // autonSchedule.enqueue<move_pose_c>(
    //     lemlib::Pose{ mx * 0_cm, my * 122_cm, adjustHeading(270, mx, my) },
    //     chassis,
    //     poseCFG{
    //         .reversed = true, .minSpeed = 25, .maxSpeed = 25, .timeout = 40
    //         });

    autonSchedule.enqueue<mech_state_c>(OUT_HOARD, outtake);
    // autonSchedule.enqueue<move_pose_c>(
    //     lemlib::Pose{ mx * 20000_cm, my * 120_cm, adjustHeading(270, mx, my)
    //     }, chassis, poseCFG{ .reversed = false,
    //              .minSpeed = 30,
    //              .maxSpeed = 40,
    //              .timeout = 500 });
    for (int i = 0; i < (doSkills ? 10 : 3); i++) {
        autonSchedule.enqueue<move_pose_c>(
            lemlib::Pose{
                mx * 999_cm, my * 110_cm, adjustHeading(270, mx, my) },
            chassis,
            poseCFG{ .reversed = false,
                     .minSpeed = 55,
                     .maxSpeed = 55,
                     .timeout = 100 });
        autonSchedule.enqueue<move_pose_c>(
            lemlib::Pose{
                mx * 999_cm, my * 110_cm, adjustHeading(270, mx, my) },
            chassis,
            poseCFG{ .reversed = false,
                     .minSpeed = 30,
                     .maxSpeed = 30,
                     .timeout = 400 });
    }
    // autonSchedule.enqueue<wait_c>(doSkills ? 2000 : 1500);

    // autonSchedule.enqueue<setPose_c>(
    //     lemlib::Pose{ mx * float(170_cm), my * float(120_cm), 9999 });
    autonSchedule.enqueue<move_pose_c>(
        lemlib::Pose{ mx * 98_cm, my * 110_cm, adjustHeading(270, mx, my) },
        chassis,
        poseCFG{ .reversed = true, .minSpeed = 80 });
    autonSchedule.enqueue<turn_heading_c>(adjustHeading(270, mx, my), chassis);
    autonSchedule.enqueue<move_pose_c>(
        lemlib::Pose{ mx * 71_cm, my * 110_cm, adjustHeading(270, mx, my) },
        chassis,
        poseCFG{ .reversed = true, .maxSpeed = 90, .timeout = 900 });

    autonSchedule.enqueue<mech_state_c>(OUT_TOP, outtake);
    autonSchedule.enqueue<togglePneu_c>(scraper);
    // autonSchedule.enqueue<move_pose_c>(
    //     lemlib::Pose{ mx * 0_cm, my * 121_cm, adjustHeading(270, mx, my) },
    //     chassis,
    //     poseCFG{ .reversed = true, .maxSpeed = 40, .timeout = 1500 });
    autonSchedule.enqueue<wait_c>(doSkills ? 5000 : 500);
    autonSchedule.enqueue<mech_state_c>(OUT_BOTTOM, outtake);
    autonSchedule.enqueue<wait_c>(doSkills ? 75 : 75);

    autonSchedule.enqueue<mech_state_c>(OUT_TOP, outtake);
    autonSchedule.enqueue<wait_c>(doSkills ? 3000 : 1000);

    autonSchedule.enqueue<outtake_toggle_c>(false, outtake);
    autonSchedule.enqueue<togglePneu_c>(descorer_l);

    autonSchedule.enqueue<move_pose_c>(
        lemlib::Pose{ mx * 3000_cm, my * 121_cm, adjustHeading(270, mx, my) },
        chassis,
        poseCFG{ .reversed = false,
                 .minSpeed = 110,
                 .maxSpeed = 127,
                 .timeout = 300,
                 .earlyExitRange = 3 });
    // autonSchedule.enqueue<turn_heading_c>(adjustHeading(220, mx, my),
    // chassis);
    //  autonSchedule.enqueue<setPose_c>(
    //      lemlib::Pose{ mx * 130_cm, my * 120_cm, adjustHeading(270, mx, my)
    //      });
    if (doSkills) {
        autonSchedule.enqueue<move_pose_c>(
            lemlib::Pose{ mx * 110_cm, my * 80_cm, adjustHeading(180, mx, my) },
            chassis,
            poseCFG{ .reversed = false,
                     .minSpeed = 127,
                     .maxSpeed = 127,
                     .timeout = 9999 });
        autonSchedule.enqueue<move_pose_c>(
            lemlib::Pose{ mx * -10_cm, my * 60_cm, adjustHeading(270, mx, my) },
            chassis,
            poseCFG{ .reversed = true,
                     .minSpeed = 100,
                     .maxSpeed = 100,
                     .timeout = 9999 });
        autonSchedule.enqueue<move_pose_c>(
            lemlib::Pose{ mx * 80_cm, my * 40_cm, adjustHeading(270, mx, my) },
            chassis,
            poseCFG{ .reversed = false,
                     .minSpeed = 100,
                     .maxSpeed = 100,
                     .timeout = 9999 });
        autonSchedule.enqueue<move_pose_c>(
            lemlib::Pose{ mx * 130_cm, my * -6_cm, adjustHeading(270, mx, my) },
            chassis,
            poseCFG{ .reversed = false,
                     .minSpeed = 100,
                     .maxSpeed = 100,
                     .timeout = 9999 });
        autonSchedule.enqueue<move_pose_c>(
            lemlib::Pose{ mx * 95, my * -10_cm, adjustHeading(270, mx, my) },
            chassis,
            poseCFG{ .reversed = false,
                     .minSpeed = 100,
                     .maxSpeed = 100,
                     .timeout = 9999 });
        autonSchedule.enqueue<move_pose_c>(
            lemlib::Pose{ mx * 3000, my * -10_cm, adjustHeading(270, mx, my) },
            chassis,
            poseCFG{ .reversed = false,
                     .minSpeed = 127,
                     .maxSpeed = 127,
                     .timeout = 9999 });
    }
    autonSchedule.enqueue<move_pose_c>(
        lemlib::Pose{ mx * 90_cm, my * 147_cm, adjustHeading(270, mx, my) },
        chassis,
        poseCFG{ .reversed = true,
                 .minSpeed = 100,
                 .maxSpeed = 127,
                 .timeout = 1300 });

    // autonSchedule.enqueue<brake_c>(chassis, pros::E_MOTOR_BRAKE_BRAKE);
    autonSchedule.enqueue<move_pose_c>(
        lemlib::Pose{ mx * 33_cm, my * 146_cm, adjustHeading(270, mx, my) },
        chassis,
        poseCFG{ .reversed = true,
                 .minSpeed = 70,
                 .maxSpeed = 70,
                 .timeout = 1800 });
    // autonSchedule.enqueue<wait_c>(doSkills ? 30 : 30);
    // autonSchedule.enqueue<togglePneu_c>(descorer_l);

    // autonSchedule.enqueue<move_pose_c>(
    //     lemlib::Pose{ mx * 800_cm, my * 146_cm, adjustHeading(270, mx, my) },
    //     chassis,
    //     poseCFG{ .reversed = false,
    //              .minSpeed = 127,
    //              .maxSpeed = 127,
    //              .timeout = 500 });
    // autonSchedule.enqueue<brake_c>(chassis, pros::E_MOTOR_BRAKE_COAST);

    while (autonSchedule.tick()) {
        pros::delay(10);
        // std::cout << chassis.getPose().x << std::endl;
    }
    std::cout << double(pros::millis() - iteration) / 1000 << std::endl;
    std::cout << chassis.getPose().x * 2.54 << std::endl;
    std::cout << chassis.getPose().y * 2.54 << std::endl;
}

void auton_full(bool isBlue) {
    float mx = isBlue ? 1 : -1;
    float my = isBlue ? 1 : -1;
    int iteration = pros::millis();
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    // chassis.setPose(mx * 124_cm, my * -48_cm, adjustHeading(90, mx, my));
    autonSchedule.enqueue<setPose_c>(lemlib::Pose{
        mx * float(118.8_cm), my * float(38.1_cm), adjustHeading(90, mx, my) });
    autonSchedule.enqueue<movepose_c>(
        lemlib::Pose{ 60, 77, 0 },
        chassis,
        mx,
        my,
        poseCFG{ .timeout = 600, .earlyExitRange = 3 });
    autonSchedule.enqueue<mech_state_c>(OUT_HOARD, outtake);
    autonSchedule.enqueue<movepose_c>(
        lemlib::Pose{ 60, 77, 0 },
        chassis,
        mx,
        my,
        poseCFG{ .minSpeed = 80, .maxSpeed = 100, .earlyExitRange = 3 });
    autonSchedule.enqueue<movepoint_c>(
        point{ 131, 104 },
        chassis,
        mx,
        my,
        poseCFG{ .minSpeed = 80, .timeout = 400, .earlyExitRange = 4 });
    autonSchedule.enqueue<movepoint_c>(
        point{ 120, 120 },
        chassis,
        mx,
        my,
        poseCFG{ .minSpeed = 50, .earlyExitRange = 4 });

    // autonSchedule.enqueue<movepose_c>(
    //     lemlib::Pose{ 130, 120, 270 }, chassis, mx, my, poseCFG{});
    autonSchedule.enqueue<togglePneu_c>(scraper);
    autonSchedule.enqueue<movepose_c>(lemlib::Pose{ 130, 120, 270 },
                                      chassis,
                                      mx,
                                      my,
                                      poseCFG{ .earlyExitRange = 2 });

    // autonSchedule.enqueue<turnheading_c>(
    //     270, chassis, mx, my, poseCFG{ .minSpeed = 5, .timeout = 1000 });
    // autonSchedule.enqueue<setPose_c>(lemlib::Pose{
    //     mx * float(120_cm), my * 121_cm, adjustHeading(9999, mx, my) });
    autonSchedule.enqueue<movepose_c>(
        lemlib::Pose{ 180, 120, 270 },
        chassis,
        mx,
        my,
        poseCFG{ .minSpeed = 80, .maxSpeed = 80, .timeout = 600 });
    autonSchedule.enqueue<movepoint_c>(
        point{ 999, 120 },
        chassis,
        mx,
        my,
        poseCFG{ .minSpeed = 40, .maxSpeed = 60, .timeout = 1200 });

    autonSchedule.enqueue<movepose_c>(
        lemlib::Pose{ 70, 120, 270 },
        chassis,
        mx,
        my,
        poseCFG{ .reversed = true, .minSpeed = 50, .timeout = 800 });
    autonSchedule.enqueue<mech_state_c>(OUT_TOP, outtake);
    autonSchedule.enqueue<wait_c>(1000);
    autonSchedule.enqueue<outtake_toggle_c>(false, outtake);
    autonSchedule.enqueue<togglePneu_c>(scraper);
    autonSchedule.enqueue<movepoint_c>(
        point{ 999, 120 },
        chassis,
        mx,
        my,
        poseCFG{ .minSpeed = 127, .maxSpeed = 127, .timeout = 100 });

    autonSchedule.enqueue<movepose_c>(lemlib::Pose{ 90, 0, 180 },
                                      chassis,
                                      mx,
                                      my,
                                      poseCFG{ .minSpeed = 127 });
    autonSchedule.enqueue<mech_state_c>(OUT_HOARD, outtake);

    autonSchedule.enqueue<movepoint_c>(
        point{ 51, -60 }, chassis, mx, my, poseCFG{ .minSpeed = 100 });
    autonSchedule.enqueue<movepose_c>(
        lemlib::Pose{ 46, -46, 225 },
        chassis,
        mx,
        my,
        poseCFG{ .reversed = true, .minSpeed = 60 });
    autonSchedule.enqueue<movepose_c>(
        lemlib::Pose{ 35, -35, 225 },
        chassis,
        mx,
        my,
        poseCFG{ .reversed = true, .minSpeed = 20 });
    autonSchedule.enqueue<mech_state_c>(OUT_MIDDLE, outtake);

    // autonSchedule.enqueue<movepose_c>(
    //     lemlib::Pose{ 35, -35, 225 },
    //     chassis,
    //     mx,
    //     my,
    //     poseCFG{ .reversed = true, .minSpeed = 20 });

    autonSchedule.enqueue<turnheading_c>(
        225, chassis, mx, my, poseCFG{ .minSpeed = 30 });
    autonSchedule.enqueue<wait_c>(500);
    autonSchedule.enqueue<outtake_toggle_c>(false, outtake);
    /*
    autonSchedule.enqueue<movepoint_c>(
        point{ 118, -120 }, chassis, mx, my, poseCFG{ .minSpeed = 80 });
    autonSchedule.enqueue<turnheading_c>(270, chassis, mx, my, poseCFG{});
    autonSchedule.enqueue<togglePneu_c>(scraper);

    autonSchedule.enqueue<movepoint_c>(
        point{ 155, -120 }, chassis, mx, my, poseCFG{ .minSpeed = 80 });
    autonSchedule.enqueue<wait_c>(1000);

    autonSchedule.enqueue<togglePneu_c>(scraper);

    autonSchedule.enqueue<movepose_c>(lemlib::Pose{ 70, -120, 270 },
                                      chassis,
                                      mx,
                                      my,
                                      poseCFG{ .reversed = true });
    autonSchedule.enqueue<mech_state_c>(OUT_TOP, outtake);
    autonSchedule.enqueue<wait_c>(1000);
    autonSchedule.enqueue<outtake_toggle_c>(false, outtake);*/
    while (autonSchedule.tick()) {
        pros::delay(10);
        // std::cout << chassis.getPose().x << std::endl;
    }
    std::cout << double(pros::millis() - iteration) / 1000 << std::endl;
    std::cout << chassis.getPose().x * 2.54 << std::endl;
    std::cout << chassis.getPose().y * 2.54 << std::endl;
}

void auton_skills() {
    autonSchedule.enqueue<setPose_c>(lemlib::Pose{ -116.2_cm, -42.916_cm, 80 });
    autonSchedule.enqueue<move_pose_c>(lemlib::Pose{ -139, -120, 270 },
                                       chassis);
    autonSchedule.enqueue<togglePneu_c>(scraper);
    autonSchedule.enqueue<move_pose_c>(lemlib::Pose{ -158, -120, 270 },
                                       chassis);
    autonSchedule.enqueue<mech_state_c>(OUT_HOARD, outtake);
    autonSchedule.enqueue<wait_c>(1500);
    autonSchedule.enqueue<move_pose_c>(lemlib::Pose{ -80, -120, 270 }, chassis);
}

void testing_pid() {
    chassis.setPose(0, 0, 0);
    chassis.moveToPose(0, 24, 0, 9999);
    std::cout << chassis.getPose().x << std::endl;
    std::cout << chassis.getPose().y << std::endl;
    // chassis.turnToHeading(90, 99999);
}
