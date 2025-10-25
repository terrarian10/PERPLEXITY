#include "autons.hpp"
#include "commandHandling.hpp"
#include "consts.h"
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

// Skills auton
// void auton_skills(float mx, float my) {
//     uint32_t start_time = pros::millis();

//     int iteration = pros::millis();
//     chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
//     chassis.setPose(mx * 160_cm, my * 48.5_cm, adjustHeading(90, mx, my));
//     autonSchedule.enqueue<move_point_c>(
//         mx * 140_cm, my * 48.5_cm, chassis, poseCFG{ false, 120 });
//     autonSchedule.enqueue<move_pose_c>(
//         lemlib::Pose{ mx * 105_cm, my * 64_cm, adjustHeading(90, mx, my) },
//         chassis,
//         poseCFG{ false, 100, 127 });

//     autonSchedule.enqueue<outtake_c>(Outt_States::BOTTOM_STORE, outtake);
//     autonSchedule.enqueue<move_pose_c>(
//         lemlib::Pose{ mx * 40_cm, my * 56_cm, adjustHeading(90, mx, my) },
//         chassis,
//         poseCFG{ false, 20, 80 });
//     autonSchedule.enqueue<move_pose_c>(
//         lemlib::Pose{ mx * 56_cm, my * 56_cm, adjustHeading(90, mx, my) },
//         chassis,
//         poseCFG{ .reversed = false });
//     autonSchedule.enqueue<turn_heading_c>(adjustHeading(135, mx, my),
//     chassis); if ((mx == 1 && my == 1) || (mx == -1 && my == -1)) {
//         autonSchedule.enqueue<move_point_c>(
//             mx * 41.5_cm, my * 45.5_cm, chassis, poseCFG{ false, 40 });

//         autonSchedule.enqueue<move_pose_c>(
//             lemlib::Pose{
//                 mx * float(46_cm),
//                 my * float(46_cm),
//                 adjustHeading(135, mx, my),
//             },
//             chassis,
//             poseCFG{ true, 5, 127, 1500, 0 });
//         autonSchedule.enqueue<move_pose_c>(
//             lemlib::Pose{
//                 mx * float(46_cm),
//                 my * float(46_cm),
//                 adjustHeading(135, mx, my),
//             },
//             chassis,
//             poseCFG{ true, 5, 127, 1500, 0 });
//     } else {
//         // autonSchedule.enqueue<move_point_c>(
//         //     mx * 45.5_cm, my * 45.5_cm, chassis, poseCFG{ false, 40 });
//         // autonSchedule.enqueue<move_point_c>(
//         //     mx * 45.5_cm, my * 45.5_cm, chassis, poseCFG{ false, 40 });
//         // autonSchedule.enqueue<move_point_c>(
//         //     mx * 45.5_cm, my * 45.5_cm, chassis, poseCFG{ false, 40 });

//         // autonSchedule.enqueue<move_pose_c>(
//         //     lemlib::Pose{
//         //         mx * (48_cm),
//         //         my * (48_cm),
//         //         adjustHeading(135, mx, my),
//         //     },
//         //     chassis,
//         //     poseCFG{ true, 20, 127, 2000 });
//         // autonSchedule.enqueue<move_pose_c>(
//         //     lemlib::Pose{
//         //         mx * (48_cm),
//         //         my * (48_cm),
//         //         adjustHeading(135, mx, my),
//         //     },
//         //     chassis,
//         //     poseCFG{ true, 20, 127, 2000 });
//         // autonSchedule.enqueue<move_pose_c>(
//         //     lemlib::Pose{
//         //         mx * float(48_cm),
//         //         my * float(48_cm),
//         //         adjustHeading(135, mx, my),
//         //     },
//         //     chassis,
//         //     poseCFG{ true, 20, 127, 1500, 0 });
//     }

//     if ((mx == 1 && my == 1) || (mx == -1 && my == -1)) {
//         autonSchedule.enqueue<outtake_c>(Outt_States::TOP_STORE, outtake);
//         autonSchedule.enqueue<wait_c>(500);
//     }
//     // autonSchedule.enqueue<move_pose_c>(
//     //     lemlib::Pose{
//     //         mx * (48_cm),
//     //         my * (48_cm),
//     //         adjustHeading(135, mx, my),
//     //     },
//     //     chassis,
//     //     poseCFG{ true, 20, 127, 2000 });
//     autonSchedule.enqueue<outtake_c>(Outt_States::MIDDLE, outtake);
//     autonSchedule.enqueue<wait_c>(2000); // Wait to like outtake
//                                          // Go to matchload

//     autonSchedule.enqueue<outtake_c>(Outt_States::OFF, outtake);
//     autonSchedule.enqueue<move_pose_c>(
//         lemlib::Pose{ mx * 50_cm, my * 50_cm, adjustHeading(135, mx, my) },
//         chassis,
//         poseCFG{ true });
//     if ((mx == 1 && my == 1) || (mx == -1 && my == -1)) {

//     } else {
//     }
//     autonSchedule.enqueue<move_pose_c>(
//         lemlib::Pose{ mx * 110_cm, my * 110_cm, adjustHeading(135, mx, my) },
//         chassis,
//         poseCFG{ .reversed = true, .minSpeed = 100, .earlyExitRange = 0.5 });
//     autonSchedule.enqueue<togglePneu_c>(scraper);

//     autonSchedule.enqueue<move_pose_c>(
//         lemlib::Pose{ mx * 130_cm, my * 121_cm, adjustHeading(270, mx, my) },
//         chassis,
//         poseCFG{ .reversed = false, .minSpeed = 100 });

//     autonSchedule.enqueue<move_pose_c>(
//         lemlib::Pose{ mx * 135_cm, my * 121_cm, 270 }, chassis);
//     autonSchedule.enqueue<outtake_c>(Outt_States::BOTTOM_STORE, outtake);

//     autonSchedule.enqueue<move_pose_c>(
//         lemlib::Pose{
//             mx * float(170_cm), my * 121_cm, adjustHeading(270, mx, my) },
//         chassis,
//         poseCFG{ false, 100, 127, 1000 });
//     autonSchedule.enqueue<move_pose_c>(
//         lemlib::Pose{
//             mx * float(170_cm), my * 121_cm, adjustHeading(270, mx, my) },
//         chassis,
//         poseCFG{ false, 100, 127, 500 });
//     autonSchedule.enqueue<move_point_c>(mx * float(190.8_cm),
//                                         my * 121_cm,
//                                         chassis,
//                                         poseCFG{ false, 100, 127, 500 });
//     autonSchedule.enqueue<setPose_c>(
//         lemlib::Pose{ mx * 170_cm, 121_cm * my, adjustHeading(270, mx, my)
//         });

//     autonSchedule.enqueue<wait_c>(500);
//     autonSchedule.enqueue<move_point_c>(
//         mx * 120_cm, my * 120_cm, chassis, poseCFG{ true, 70 });
//     autonSchedule.enqueue<move_point_c>(
//         mx * 106_cm,
//         my * 122_cm,
//         chassis,
//         poseCFG{ .reversed = true, .maxSpeed = 70, .timeout = 2500 });
//     autonSchedule.enqueue<outtake_c>(Outt_States::TOP, outtake);
//     autonSchedule.enqueue<wait_c>(300);

//     autonSchedule.enqueue<outtake_c>(Outt_States::BOTTOM_STORE, outtake);
//     autonSchedule.enqueue<wait_c>(300);

//     autonSchedule.enqueue<outtake_c>(Outt_States::TOP, outtake);
//     autonSchedule.enqueue<wait_c>(2500);
//     autonSchedule.enqueue<outtake_c>(Outt_States::OFF, outtake);
//     autonSchedule.enqueue<move_point_c>(
//         mx * 160_cm,
//         my * 0_cm,
//         chassis,
//         poseCFG{ .reversed = true,
//                  .maxSpeed = 70,
//                  .timeout = 2500 }); // Outputs total time, for testing
//     while (autonSchedule.tick()) {
//         pros::delay(10);
//         // std::cout << chassis.getPose().x << std::endl
//     }
//     std::cout << double(pros::millis() - iteration) / 1000 << std::endl;
//     std::cout << chassis.getPose().x * 2.54 << std::endl;
//     std::cout << chassis.getPose().y * 2.54 << std::endl;
//     std::cout << "\033[7mTIME time: " << (pros::millis() - start_time) /
//     1000.0
//               << " sec\033[0m\n";
// }

// Individial side autons
void autons_positive_red() {
    // int iteration = pros::millis();
    // chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    // chassis.setPose(-156_cm, -42.6_cm, 90);
    // // chassis.setPose(chassis.getPose().x, chassis.getPose().y, 90);
    // //  Go to short goal
    // autonSchedule.enqueue<move_pose_c>(lemlib::Pose{ -90_cm, -42.6_cm, 90 },
    //                                    chassis);
    // autonSchedule.enqueue<wait_c>(500);
    // autonSchedule.enqueue<outtake_c>(Outt_States::BOTTOM_STORE, outtake);
    // autonSchedule.enqueue<wait_c>(0);
    // autonSchedule.enqueue<move_pose_c>(lemlib::Pose{ -50_cm, -50_cm, 45 },
    //                                    chassis);
    // autonSchedule.enqueue<outtake_c>(Outt_States::OFF, outtake);

    // autonSchedule.enqueue<wait_c>(500);
    // autonSchedule.enqueue<move_pose_c>(
    //     lemlib::Pose{ -25_cm, -25_cm, 45 }, chassis, false);
    // autonSchedule.enqueue<outtake_c>(Outt_States::BOTTOM, outtake);

    // autonSchedule.enqueue<wait_c>(2000); // Wait to like outtake
    //                                      // Go to matchload
    // autonSchedule.enqueue<outtake_c>(Outt_States::OFF, outtake);

    // autonSchedule.enqueue<move_pose_c>(
    //     lemlib::Pose{ -30, -25, 45 }, chassis, true);
    // autonSchedule.enqueue<move_pose_c>(lemlib::Pose{ -124_cm, -121_cm, 270 },
    //                                    chassis);
    // autonSchedule.enqueue<togglePneu_c>(scraper); // Toggle Scraper

    // autonSchedule.enqueue<move_pose_c>(
    //     lemlib::Pose{ -144_cm, -124_cm, 270 },
    //     chassis); // Goes to like the match load thingimabober
    // autonSchedule.enqueue<outtake_c>(Outt_States::BOTTOM_STORE, outtake);

    // autonSchedule.enqueue<wait_c>(2000);
    // autonSchedule.enqueue<outtake_c>(Outt_States::OFF, outtake);
    // autonSchedule.enqueue<togglePneu_c>(scraper); // Toggle Scraper

    // autonSchedule.enqueue<move_pose_c>(
    //     lemlib::Pose{ -60_cm, -116_cm, 270 }, chassis, true);
    // autonSchedule.enqueue<wait_c>(1500);

    // autonSchedule.enqueue<move_pose_c>(
    //     lemlib::Pose{ -100_cm, -157_cm, 270 }, chassis, false);
    // autonSchedule.enqueue<togglePneu_c>(descorer);
    // autonSchedule.enqueue<move_pose_c>(
    //     lemlib::Pose{ -10_cm, -157_cm, 270 }, chassis, true);

    // chassis.moveToPose(-100_cm, -45_cm, 90, 5000);
    // outtake.run_at_state(Outt_States::BOTTOM);

    // chassis.moveToPose(-45_cm, -60_cm, 90, 5000);
    //  chassis.moveToPose(-30_cm, -45_cm, 225, 5000);
    //  chassis.moveToPose(-140_cm, -121_cm, 270, 5000);
    //  chassis.moveToPose(-90_cm, -121_cm, 270, 5000);

    // chassis.turnToHeading(180, 5000);
    // chassis.waitUntilDone();
    // chassis.turnToHeading(360, 5000);

    while (autonSchedule.tick()) {
        pros::delay(10);
        // std::cout << chassis.getPose().x << std::endl;
    }
    // std::cout << double(pros::millis() - iteration) / 1000 << std::endl;
}

void autons_positive_blue() {}

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
        lemlib::Pose{ mx * 105_cm, my * 57_cm, adjustHeading(90, mx, my) },
        chassis,
        poseCFG{ false, 100, 127 });

    autonSchedule.enqueue<outtake_c>(Outt_States::BOTTOM_STORE, outtake);
    autonSchedule.enqueue<move_pose_c>(
        lemlib::Pose{ mx * 40_cm,
                      my * (doSkills ? 60_cm : 60_cm),
                      adjustHeading(90, mx, my) },
        chassis,
        poseCFG{ false, 20, doSkills ? 80 : 80 });
    autonSchedule.enqueue<move_pose_c>(
        lemlib::Pose{ mx * 60_cm, my * 60_cm, adjustHeading(90, mx, my) },
        chassis,
        poseCFG{ .reversed = false });
    autonSchedule.enqueue<turn_heading_c>(adjustHeading(135, mx, my), chassis);
    // autonSchedule.enqueue<move_pose_c>(lemlib::Pose{ -44_cm, 40_cm, 135 },
    //                                    chassis);
    // autonSchedule.enqueue<move_point_c>(
    //     mx * 60_cm, my * 60_cm, chassis, poseCFG{ true });
    // autonSchedule.enqueue<move_pose_c>(
    //     lemlib::Pose{ mx * 60_cm, my * 60_cm, 135 }, chassis, poseCFG{ true
    //     });

    // autonSchedule.enqueue<outtake_c>(Outt_States::OFF, outtake);

    // autonSchedule.enqueue<wait_c>(200);
    if ((mx == 1 && my == 1) || (mx == -1 && my == -1)) {
        autonSchedule.enqueue<move_point_c>(
            mx * 41.5_cm, my * 45.5_cm, chassis, poseCFG{ false, 40 });

        autonSchedule.enqueue<move_pose_c>(
            lemlib::Pose{
                mx * float(43_cm),
                my * float(43_cm),
                adjustHeading(135, mx, my),
            },
            chassis,
            poseCFG{ false, 5, 127, 1500, 0 });
    } else {
        autonSchedule.enqueue<move_point_c>(
            mx * 40_cm, my * 40_cm, chassis, poseCFG{ false, 40 });

        autonSchedule.enqueue<move_pose_c>(
            lemlib::Pose{
                mx * float(39_cm),
                my * float(39_cm),
                adjustHeading(135, mx, my),
            },
            chassis,
            poseCFG{ false, 5, 127, 1500, 0 });
    }

    // autonSchedule.enqueue<move_pose_c>(
    //     lemlib::Pose{
    //         mx * float(42.5_cm),
    //         my * float(42.5_cm),
    //         135,
    //     },
    //     chassis,
    //     poseCFG{ true, 5, 127, 5000, 0 });

    // autonSchedule.enqueue<outtake_c>(Outt_States::BOTTOM_STORE, outtake);
    // autonSchedule.enqueue<wait_c>(250);
    autonSchedule.enqueue<outtake_c>(Outt_States::TOP_STORE, outtake);
    autonSchedule.enqueue<wait_c>(250);
    if ((mx == 1 && my == 1) || (mx == -1 && my == -1)) {
        autonSchedule.enqueue<outtake_c>(Outt_States::TOP_STORE, outtake);
        autonSchedule.enqueue<wait_c>(500);
    }
    autonSchedule.enqueue<wait_c>(250);
    autonSchedule.enqueue<outtake_c>(Outt_States::MIDDLE, outtake);
    autonSchedule.enqueue<wait_c>(doSkills ? 3000
                                           : 2000); // Wait to like outtake
                                                    // Go to matchload
    // autonSchedule.enqueue<outtake_c>(Outt_States::BOTTOM_STORE, outtake);
    // autonSchedule.enqueue<wait_c>(250);
    // autonSchedule.enqueue<outtake_c>(Outt_States::MIDDLE, outtake);
    // autonSchedule.enqueue<wait_c>(250);
    autonSchedule.enqueue<outtake_c>(Outt_States::OFF, outtake);
    autonSchedule.enqueue<move_pose_c>(
        lemlib::Pose{ mx * 46_cm, my * 46_cm, adjustHeading(135, mx, my) },
        chassis,
        poseCFG{ true });
    if ((mx == 1 && my == 1) || (mx == -1 && my == -1)) {
        // autonSchedule.enqueue<move_pose_c>(
        //     lemlib::Pose{
        //         mx * 120_cm, my * 121_cm, adjustHeading(135, mx, my) },
        //     chassis,
        //     poseCFG{
        //         .reversed = true, .minSpeed = 100, .earlyExitRange = 0.5 });
        // autonSchedule.enqueue<move_pose_c>(
        //     lemlib::Pose{
        //         mx * 120_cm, my * 121_cm, adjustHeading(270, mx, my) },
        //     chassis,
        //     poseCFG{
        //         .reversed = true, .minSpeed = 100, .earlyExitRange = 0.5 });
        // autonSchedule.enqueue<move_pose_c>(
        //     lemlib::Pose{ mx * 130_cm, my * 90_cm, adjustHeading(180, mx, my)
        //     }, chassis, poseCFG{
        //         .reversed = true, .minSpeed = 100, .earlyExitRange = 0.5 });
        // autonSchedule.enqueue<move_pose_c>(
        //     lemlib::Pose{ mx * 130_cm, my * 90_cm, adjustHeading(200, mx, my)
        //     }, chassis, poseCFG{
        //         .reversed = true, .minSpeed = 100, .earlyExitRange = 0.5 });
    } else {
    }
    autonSchedule.enqueue<move_pose_c>(
        lemlib::Pose{ mx * 120_cm, my * 116_cm, adjustHeading(135, mx, my) },
        chassis,
        poseCFG{ .reversed = true, .minSpeed = 50, .earlyExitRange = 0.5 });
    autonSchedule.enqueue<togglePneu_c>(scraper);

    autonSchedule.enqueue<turn_heading_c>(
        adjustHeading(270, mx, my),
        chassis,
        poseCFG{ .reversed = true, .minSpeed = 100, .earlyExitRange = 0.5 });
    autonSchedule.enqueue<turn_heading_c>(
        adjustHeading(270, mx, my),
        chassis,
        poseCFG{ .reversed = true, .minSpeed = 100, .earlyExitRange = 0.5 });
    autonSchedule.enqueue<turn_heading_c>(
        adjustHeading(270, mx, my),
        chassis,
        poseCFG{ .reversed = true, .minSpeed = 100, .earlyExitRange = 0.5 });
    autonSchedule.enqueue<turn_heading_c>(
        adjustHeading(270, mx, my),
        chassis,
        poseCFG{ .reversed = true, .minSpeed = 100, .earlyExitRange = 0.5 });
    autonSchedule.enqueue<turn_heading_c>(
        adjustHeading(270, mx, my),
        chassis,
        poseCFG{ .reversed = true, .minSpeed = 40, .earlyExitRange = 0.5 });
    // autonSchedule.enqueue<wait_c>(200);
    //  autonSchedule.enqueue<turn_heading_c>(adjustHeading(270, mx, my),
    //  chassis);
    autonSchedule.enqueue<setPose_c>(lemlib::Pose{
        mx * 120_cm, float(121_cm) * my, adjustHeading(270, mx, my) });
    // autonSchedule.enqueue<move_pose_c>(
    //     lemlib::Pose{ mx * 140_cm, my * 121_cm, adjustHeading(270, mx, my) },
    //     chassis,
    //     poseCFG{ .reversed = false, .minSpeed = 100, .earlyExitRange = 1 });
    // autonSchedule.enqueue<move_point_c>(
    //     mx * 150_cm, my * 121_cm, chassis, poseCFG{ .minSpeed = 100 });

    // autonSchedule.enqueue<move_point_c>(
    //     mx * 160_cm, my * 121_cm, chassis, poseCFG{ .minSpeed = 100 });
    // autonSchedule.enqueue<move_pose_c>(
    //     lemlib::Pose{ mx * 120_cm, my * 121_cm, 270 },
    //     chassis,
    //     poseCFG{ true });
    // autonSchedule.enqueue<move_pose_c>(
    //     lemlib::Pose{ mx * 120_cm, my * 121_cm, 270 },
    //     chassis,
    //     poseCFG{ false });

    autonSchedule.enqueue<move_pose_c>(
        lemlib::Pose{ mx * 145_cm, my * 121_cm, adjustHeading(270, mx, my) },
        chassis,
        poseCFG{ .minSpeed = 90 });

    for (int g = 0; g <= (doSkills ? 0 : 0); g++) {
        autonSchedule.enqueue<outtake_c>(Outt_States::BOTTOM_STORE,
                                         outtake); // MATCH LOAD FROM MATCH LOAD

        autonSchedule.enqueue<move_pose_c>(
            lemlib::Pose{
                mx * float(170_cm), my * 122_cm, adjustHeading(270, mx, my) },
            chassis,
            poseCFG{ false, 100, 127, 500 });
        autonSchedule.enqueue<move_pose_c>(
            lemlib::Pose{
                mx * float(170_cm), my * 122_cm, adjustHeading(270, mx, my) },
            chassis,
            poseCFG{ false, 100, 127, 500 });
        autonSchedule.enqueue<move_point_c>(mx * float(190.8_cm),
                                            my * 122_cm,
                                            chassis,
                                            poseCFG{ false, 100, 127, 150 });

        // autonSchedule.enqueue<setPose_c>(lemlib::Pose{
        //     mx * 170_cm, 120_cm * my, adjustHeading(270, mx, my) });
        autonSchedule.enqueue<move_point_c>(
            mx * 169_cm, 120_cm * my, chassis, poseCFG{ true, 100 });
        autonSchedule.enqueue<move_point_c>(mx * 169_cm,
                                            120_cm * my,
                                            chassis,
                                            poseCFG{ .reversed = false,
                                                     .minSpeed = 40,
                                                     .maxSpeed = 80,
                                                     .timeout = 150 });
        // autonSchedule.enqueue<move_pose_c>(
        //     lemlib::Pose{ mx * 800_cm, my * 140_cm, 270 },
        //     chassis,
        //     poseCFG{ false, 80, 100, 250 }); // Wait to like intake and
        //                                      // not-so-slowly move

        autonSchedule.enqueue<wait_c>(500);
        autonSchedule.enqueue<move_pose_c>(
            lemlib::Pose{
                mx * 120_cm, my * 120_cm, adjustHeading(270, mx, my) },
            chassis,
            poseCFG{ true, 70 });
        autonSchedule.enqueue<move_point_c>(
            mx * 106_cm,
            my * 122_cm,
            chassis,
            poseCFG{ .reversed = true, .maxSpeed = 70, .timeout = 2500 });
        autonSchedule.enqueue<outtake_c>(Outt_States::TOP, outtake);
        autonSchedule.enqueue<move_point_c>(
            mx * 90_cm,
            my * 122_cm,
            chassis,
            poseCFG{ .reversed = true, .maxSpeed = 30, .timeout = 250 });

        autonSchedule.enqueue<outtake_c>(Outt_States::UNJAM_NO_RELEASE,
                                         outtake);
        autonSchedule.enqueue<move_pose_c>(
            lemlib::Pose{ mx * 90_cm, my * 122_cm, adjustHeading(270, mx, my) },
            chassis,
            poseCFG{ .reversed = true, .maxSpeed = 30, .timeout = 500 });
        autonSchedule.enqueue<outtake_c>(Outt_States::TOP, outtake);
        for (int i = 0; i < 1; i++) {
            // autonSchedule.enqueue<outtake_c>(Outt_States::BOTTOM_STORE,
            // outtake); autonSchedule.enqueue<move_point_c>(
            //     mx * 40_cm,
            //     my * 121_cm,
            //     chassis,
            //     poseCFG{ .reversed = true, .maxSpeed = 30, .timeout = 500 });
            autonSchedule.enqueue<outtake_c>(Outt_States::TOP, outtake);
            autonSchedule.enqueue<move_point_c>(
                mx * 40_cm,
                my * 122_cm,
                chassis,
                poseCFG{ .reversed = true,
                         .maxSpeed = 30,
                         .timeout = doSkills ? 4000 : 3000 });
        }
    }
    autonSchedule.enqueue<outtake_c>(Outt_States::OFF, outtake);
    if (doSkills) {
        autonSchedule.enqueue<togglePneu_c>(scraper);
        autonSchedule.enqueue<move_point_c>(mx * 100_cm,
                                            my * 100_cm,
                                            chassis,
                                            poseCFG{ .reversed = false,
                                                     .minSpeed = 100,
                                                     .maxSpeed = 127,

                                                     .timeout = 4000 });
        autonSchedule.enqueue<move_point_c>(mx * 0_cm,
                                            my * 100_cm,
                                            chassis,
                                            poseCFG{ .reversed = false,
                                                     .minSpeed = 100,
                                                     .maxSpeed = 127,

                                                     .timeout = 4000 });
        autonSchedule.enqueue<move_point_c>(mx * 200_cm,
                                            my * -30_cm,
                                            chassis,
                                            poseCFG{ .reversed = false,
                                                     .minSpeed = 100,
                                                     .maxSpeed = 127,

                                                     .timeout = 2500 });

        autonSchedule.enqueue<move_point_c>(mx * 400_cm,
                                            my * -60_cm,
                                            chassis,
                                            poseCFG{ .reversed = false,
                                                     .minSpeed = 100,
                                                     .maxSpeed = 127,

                                                     .timeout = 2500 });
        autonSchedule.enqueue<move_point_c>(mx * 8000_cm,
                                            my * -1200_cm,
                                            chassis,
                                            poseCFG{ .reversed = false,
                                                     .minSpeed = 100,
                                                     .maxSpeed = 127,

                                                     .timeout = 2500 });
        autonSchedule.enqueue<move_point_c>(mx * 8000_cm,
                                            my * -1200_cm,
                                            chassis,
                                            poseCFG{ .reversed = false,
                                                     .minSpeed = 100,
                                                     .maxSpeed = 127,

                                                     .timeout = 2500 });
    }

    // autonSchedule.enqueue<move_pose_c>(
    //     lemlib::Pose{ mx * 105_cm, my * 121_cm, 270 },
    //     chassis,
    //     poseCFG{ .minSpeed = 40 });
    // autonSchedule.enqueue<move_pose_c>(
    //     lemlib::Pose{ mx * 140_cm, my * 85_cm, 270 }, chassis);
    // autonSchedule.enqueue<togglePneu_c>(descorer);
    // autonSchedule.enqueue<move_pose_c>(
    //     lemlib::Pose{ mx * 30_cm, my * 85_cm, 270 },
    //     chassis,
    //     poseCFG{ .reversed = true, .maxSpeed = 100 });
    // autonSchedule.enqueue<move_pose_c>(
    //     lemlib::Pose{ -30, -25, 45 }, chassis, true);
    // autonSchedule.enqueue<move_pose_c>(lemlib::Pose{ -124_cm, -121_cm, 270 },
    //                                    chassis);
    // autonSchedule.enqueue<togglePneu_c>(scraper); // Toggle ScraperH

    // autonSchedule.enqueue<move_pose_c>(
    //     lemlib::Pose{ -144_cm, -124_cm, 270 },
    //     chassis); // Goes to like the match load thingimabober
    // autonSchedule.enqueue<outtake_c>(Outt_States::BOTTOM_STORE, outtake);

    // autonSchedule.enqueue<wait_c>(2000);
    // autonSchedule.enqueue<outtake_c>(Outt_States::OFF, outtake);
    // autonSchedule.enqueue<togglePneu_c>(scraper); // Toggle Scraper

    // autonSchedule.enqueue<move_pose_c>(
    //     lemlib::Pose{ -60_cm, -116_cm, 270 }, chassis, true);
    // autonSchedule.enqueue<wait_c>(1500);

    // autonSchedule.enqueue<move_pose_c>(
    //     lemlib::Pose{ -100_cm, -157_cm, 270 }, chassis, false);
    // autonSchedule.enqueue<togglePneu_c>(descorer);
    // autonSchedule.enqueue<move_pose_c>(
    //     lemlib::Pose{ -10_cm, -157_cm, 270 }, chassis, true);

    // chassis.moveToPose(-100_cm, -45_cm, 90, 5000);
    // outtake.run_at_state(Outt_States::BOTTOM);

    // chassis.moveToPose(-45_cm, -60_cm, 90, 5000);
    //  chassis.moveToPose(-30_cm, -45_cm, 225, 5000);
    //  chassis.moveToPose(-140_cm, -121_cm, 270, 5000);
    //  chassis.moveToPose(-90_cm, -121_cm, 270, 5000);

    // chassis.turnToHeading(180, 5000);
    // chassis.waitUntilDone();
    // chassis.turnToHeading(360, 5000);
    scraper.retract();
    descorer.retract();
    while (autonSchedule.tick()) {
        pros::delay(10);
        // std::cout << chassis.getPose().x << std::endl;
    }
    std::cout << double(pros::millis() - iteration) / 1000 << std::endl;
    std::cout << chassis.getPose().x * 2.54 << std::endl;
    std::cout << chassis.getPose().y * 2.54 << std::endl;
}

void autons_negative_blue() {}
