#include "autons.hpp"
#include "commandHandling.hpp"
#include "consts.hpp"
#include "lemlib/asset.hpp"
#include "lemlib/chassis/chassis.hpp"
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
    int type = mx * my; // 1 is bottom goal -1 is midgoal
    int iteration = pros::millis();
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    // chassis.setPose(mx * 124_cm, my * -48_cm, adjustHeading(90, mx, my));
    autonSchedule.enqueue<setPose_c>(
        lemlib::Pose{ mx * float(118.8_cm),
                      my * float(38_cm),
                      adjustHeading(90, mx, my) }); // y:38
    autonSchedule.enqueue<togglePneu_c>(descorer_l);

    autonSchedule.enqueue<movepose_c>(
        lemlib::Pose{ 64, 77, 0 },
        chassis,
        mx,
        my,
        poseCFG{ .minSpeed = 105, .timeout = 675, .earlyExitRange = 3 });
    autonSchedule.enqueue<mech_state_c>(OUT_HOARD, outtake);
    autonSchedule.enqueue<togglePneu_c>(scraper);
    autonSchedule.enqueue<movepose_c>(
        lemlib::Pose{ 64, 78, 0 },
        chassis,
        mx,
        my,
        poseCFG{ .minSpeed = 85, .maxSpeed = 105 });
    // autonSchedule.enqueue<movepoint_c>(
    //     point{ 60, 60 },
    //     chassis,
    //     mx,
    //     my,
    //     poseCFG{ .minSpeed = 0, .maxSpeed = 127 });
    autonSchedule.enqueue<turnheading_c>(
        315, chassis, mx, my, poseCFG{ .timeout = 300 });
    autonSchedule.enqueue<movepoint_c>(
        point{ 49, 51 },
        chassis,
        mx,
        my,
        poseCFG{ .reversed = true, .minSpeed = 70, .maxSpeed = 80 });
    autonSchedule.enqueue<mech_state_c>(OUT_MIDDLE, outtake);

    autonSchedule.enqueue<movepoint_c>(
        point{ 35, 35 },
        chassis,
        mx,
        my,
        poseCFG{
            .reversed = true, .minSpeed = 40, .maxSpeed = 60, .timeout = 30 });
    autonSchedule.enqueue<togglePneu_c>(scraper);

    // autonSchedule.enqueue<mech_state_c>(OUT_MIDDLE, outtake);
    autonSchedule.enqueue<turnheading_c>(
        315, chassis, mx, my, poseCFG{ .timeout = 200 });
    autonSchedule.enqueue<movepoint_c>(
        point{ 33, 33 },
        chassis,
        mx,
        my,
        poseCFG{
            .reversed = true, .minSpeed = 20, .maxSpeed = 40, .timeout = 450 });

    // autonSchedule.enqueue<movepoint_c>(point{ 999, 999 },
    //                                    chassis,
    //                                    mx,
    //                                    my,
    //                                    poseCFG{ .reversed = true,
    //                                             .minSpeed = 120,
    //                                             .maxSpeed = 120,
    //                                             .timeout = 225 });
    // autonSchedule.enqueue<movepoint_c>(point{ 0, 0 },
    //                                    chassis,
    //                                    mx,
    //                                    my,
    //                                    poseCFG{ .reversed = true,
    //                                             .minSpeed = 120,
    //                                             .maxSpeed = 120,
    //                                             .timeout = 315 });
    autonSchedule.enqueue<wait_c>(1250);
    autonSchedule.enqueue<mech_state_c>(OUT_HOARD, outtake);
    autonSchedule.enqueue<movepoint_c>(
        point{ 999, 999 },
        chassis,
        mx,
        my,
        poseCFG{ .minSpeed = 127, .timeout = 200 });
    // autonSchedule.enqueue<movepoint_c>(
    //     point{ 0, 0 },
    //     chassis,
    //     mx,
    //     my,
    //     poseCFG{ .reversed = true, .minSpeed = 127, .timeout = 400 });
    autonSchedule.enqueue<movepoint_c>(
        point{ 90, 6.0 },
        chassis,
        mx,
        my,
        poseCFG{ .minSpeed = 100, .timeout = 200, .earlyExitRange = 4 });

    autonSchedule.enqueue<movepoint_c>(
        point{ 100, 112 }, chassis, mx, my, poseCFG{ .minSpeed = 75 });
    autonSchedule.enqueue<togglePneu_c>(scraper);

    // autonSchedule.enqueue<togglePneu_c>(scraper);

    // autonSchedule.enqueue<movepose_c>(
    //     lemlib::Pose{ 130, 120, 270 }, chassis, mx, my, poseCFG{});
    // autonSchedule.enqueue<movepose_c>(lemlib::Pose{ 135, 120, 270 },
    //                                   chassis,
    //                                   mx,
    //                                   my,
    //                                   poseCFG{ .earlyExitRange = 2 });

    autonSchedule.enqueue<turnheading_c>(
        270, chassis, mx, my, poseCFG{ .timeout = 600 });
    // autonSchedule.enqueue<setPose_c>(lemlib::Pose{
    //     mx * float(120_cm), my * 121_cm, adjustHeading(9999, mx, my) });
    autonSchedule.enqueue<movepoint_c>(
        point{ 180, 122 },
        chassis,
        mx,
        my,
        poseCFG{ .minSpeed = 85, .maxSpeed = 85, .timeout = 450 });
    autonSchedule.enqueue<movepoint_c>(
        point{ 180, 122 },
        chassis,
        mx,
        my,
        poseCFG{ .minSpeed = 85, .maxSpeed = 85, .timeout = 600 });

    // autonSchedule.enqueue<movepoint_c>(
    //     point{ 999, 120 },
    //     chassis,
    //     mx,
    //     my,
    //     poseCFG{ .minSpeed = 40, .maxSpeed = 60, .timeout = 50 });
    autonSchedule.enqueue<movepose_c>(
        lemlib::Pose{ 100, 123, 270 },
        chassis,
        mx,
        my,
        poseCFG{ .reversed = true, .minSpeed = 85, .timeout = 900 });

    autonSchedule.enqueue<movepose_c>(
        lemlib::Pose{ 70, 123, 270 },
        chassis,
        mx,
        my,
        poseCFG{ .reversed = true, .minSpeed = 25, .timeout = 900 });

    autonSchedule.enqueue<mech_state_c>(OUT_BOTTOM, outtake);
    autonSchedule.enqueue<movepose_c>(
        lemlib::Pose{ 0, 125, 270 },
        chassis,
        mx,
        my,
        poseCFG{
            .reversed = true, .minSpeed = 20, .maxSpeed = 40, .timeout = 150 });
    autonSchedule.enqueue<mech_state_c>(OUT_TOP, outtake);
    autonSchedule.enqueue<movepose_c>(lemlib::Pose{ 0, 120, 270 },
                                      chassis,
                                      mx,
                                      my,
                                      poseCFG{ .reversed = true,
                                               .minSpeed = 20,
                                               .maxSpeed = 40,
                                               .timeout = 2000 });
    autonSchedule.enqueue<mech_state_c>(OUT_HOARD, outtake);

    autonSchedule.enqueue<togglePneu_c>(scraper);
    autonSchedule.enqueue<movepoint_c>(
        point{ 999, 120 },
        chassis,
        mx,
        my,
        poseCFG{ .minSpeed = 127, .maxSpeed = 127, .timeout = 150 });
    autonSchedule.enqueue<movepoint_c>(
        point{ 105, float((type == 1) ? 163.0 : 92.5) },
        chassis,
        mx,
        my,
        poseCFG{ .minSpeed = 90, .maxSpeed = 127, .timeout = 1000 });
    autonSchedule.enqueue<turnheading_c>(
        270, chassis, mx, my, poseCFG{ .timeout = 600 });
    autonSchedule.enqueue<togglePneu_c>(descorer_l);
    autonSchedule.enqueue<movepoint_c>(
        point{ 37, float((type == 1) ? 163.0 : 93.5) },
        chassis,
        mx,
        my,
        poseCFG{ .reversed = true,
                 .minSpeed = 60,
                 .maxSpeed = 127,
                 .timeout = 800 });
    autonSchedule.enqueue<movepose_c>(
        lemlib::Pose{ 40, float((type == 1) ? 163.0 : 93.5), 270 },
        chassis,
        mx,
        my,
        poseCFG{ .reversed = true,
                 .minSpeed = 60,
                 .maxSpeed = 127,
                 .timeout = 1200 });
    // THIS IS SO SCUMMY WHAT DA HAIL
    autonSchedule.enqueue<swing_heading_c>(
        340,
        lemlib::DriveSide::LEFT,
        chassis,
        mx,
        my,
        swingCFG{
            .maxSpeed = 120,
            .minSpeed = 80,
            .timeout = 8000,
            .dirPath = lemlib::AngularDirection::CW_CLOCKWISE,

        });

    while (autonSchedule.tick()) {
        pros::delay(10);
        outtake.loop();

        // std::cout << chassis.getPose().x << std::endl;
    }
    std::cout << double(pros::millis() - iteration) / 1000 << std::endl;
    std::cout << chassis.getPose().x * 2.54 << std::endl;
    std::cout << chassis.getPose().y * 2.54 << std::endl;
}

void auton_full(bool isBlue, bool doSkills) {
    float mx = isBlue ? 1 : -1;
    float my = isBlue ? 1 : -1;
    int iteration = pros::millis();
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    // chassis.setPose(mx * 124_cm, my * -48_cm, adjustHeading(90, mx, my));
    autonSchedule.enqueue<setPose_c>(
        lemlib::Pose{ mx * float(119.38_cm),
                      my * float(36.28_cm),
                      adjustHeading(90, mx, my) }); // y:38
    autonSchedule.enqueue<togglePneu_c>(descorer_l);

    autonSchedule.enqueue<movepose_c>(
        lemlib::Pose{ 53, 56, 90 },
        chassis,
        mx,
        my,
        poseCFG{ .minSpeed = 105, .timeout = 700, .earlyExitRange = 3 });
    autonSchedule.enqueue<mech_state_c>(OUT_HOARD, outtake);
    autonSchedule.enqueue<togglePneu_c>(scraper);
    autonSchedule.enqueue<movepoint_c>(
        point{ 60, 56 },
        chassis,
        mx,
        my,
        poseCFG{ .minSpeed = 40, .maxSpeed = 60, .timeout = 300 });
    // autonSchedule.enqueue<movepoint_c>(point{ 64, 56 },
    //                                    chassis,
    //                                    mx,
    //                                    my,
    //                                    poseCFG{ .reversed = true,
    //                                             .minSpeed = 85,
    //                                             .maxSpeed = 105,
    //                                             .earlyExitRange = 1 });
    // MIDGOAL_START
    // autonSchedule.enqueue<turnheading_c>(
    //     135, chassis, mx, my, poseCFG{ .minSpeed = 20, .timeout = 500 });
    autonSchedule.enqueue<togglePneu_c>(scraper);

    // autonSchedule.enqueue<movepoint_c>(
    //     point{ 46, 42 },
    //     chassis,
    //     mx,
    //     my,
    //     poseCFG{ .minSpeed = 85, .maxSpeed = 105, .earlyExitRange = 1 });
    // autonSchedule.enqueue<movepose_c>(
    //     lemlib::Pose{ 39, 35, 135 },
    //     chassis,
    //     mx,
    //     my,
    //     poseCFG{ .minSpeed = 20, .maxSpeed = 105, .timeout = 500 });

    // autonSchedule.enqueue<mech_state_c>(OUT_BOTTOM, outtake);
    // autonSchedule.enqueue<wait_c>(400);
    // autonSchedule.enqueue<mech_state_c>(OUT_HOARD, outtake);

    // autonSchedule.enqueue<movepose_c>(
    //     lemlib::Pose{ 32, 32, 135 },
    //     chassis,
    //     mx,
    //     my,
    //     poseCFG{ .minSpeed = 40, .maxSpeed = 105, .timeout = 200 });
    // BOTTOM_END
    autonSchedule.enqueue<movepoint_c>(
        point{ 80, 80 },
        chassis,
        mx,
        my,
        poseCFG{ .reversed = true, .minSpeed = 127, .maxSpeed = 127 });
    autonSchedule.enqueue<movepoint_c>(
        point{ 95, 95 },
        chassis,
        mx,
        my,
        poseCFG{ .reversed = true, .minSpeed = 20, .maxSpeed = 127 });
    autonSchedule.enqueue<togglePneu_c>(scraper);

    autonSchedule.enqueue<turnheading_c>(
        270, chassis, mx, my, poseCFG{ .minSpeed = 20 });

    autonSchedule.enqueue<movepoint_c>(
        point{ 180, 120 },
        chassis,
        mx,
        my,
        poseCFG{ .minSpeed = 70,
                 .maxSpeed = 78,
                 .timeout = doSkills ? 2500 : 1000 });
    autonSchedule.enqueue<movepoint_c>(point{ 84, 120 },
                                       chassis,
                                       mx,
                                       my,
                                       poseCFG{ .reversed = true,
                                                .minSpeed = 127,
                                                .maxSpeed = 127,
                                                .timeout = 600 });
    autonSchedule.enqueue<mech_state_c>(OUT_TOP, outtake);

    autonSchedule.enqueue<movepoint_c>(
        point{ 0, 123 },
        chassis,
        mx,
        my,
        poseCFG{
            .reversed = true, .minSpeed = 40, .maxSpeed = 60, .timeout = 200 });
    autonSchedule.enqueue<mech_state_c>(OUT_BOTTOM, outtake);
    autonSchedule.enqueue<togglePneu_c>(scraper);
    autonSchedule.enqueue<movepoint_c>(
        point{ 0, 123 },
        chassis,
        mx,
        my,
        poseCFG{
            .reversed = true, .minSpeed = 40, .maxSpeed = 60, .timeout = 150 });
    autonSchedule.enqueue<mech_state_c>(OUT_TOP, outtake);

    autonSchedule.enqueue<movepoint_c>(
        point{ 0, 123 },
        chassis,
        mx,
        my,
        poseCFG{ .reversed = true,
                 .minSpeed = 40,
                 .maxSpeed = 60,
                 .timeout = doSkills ? 3000 : 1000 });
    autonSchedule.enqueue<mech_state_c>(OUT_HOARD, outtake);

    autonSchedule.enqueue<movepoint_c>(point{ 999, 123 },
                                       chassis,
                                       mx,
                                       my,
                                       poseCFG{ .reversed = false,
                                                .minSpeed = 127,
                                                .maxSpeed = 127,
                                                .timeout = 150 });
    autonSchedule.enqueue<movepoint_c>(point{ 93, 0 },
                                       chassis,
                                       mx,
                                       my,
                                       poseCFG{ .reversed = false,
                                                .minSpeed = 127,
                                                .maxSpeed = 127,
                                                .timeout = 1200 });
    autonSchedule.enqueue<movepoint_c>(point{ 56, -56 },
                                       chassis,
                                       mx,
                                       my,
                                       poseCFG{ .reversed = false,
                                                .minSpeed = 127,
                                                .maxSpeed = 127,
                                                .timeout = 650 });
    autonSchedule.enqueue<togglePneu_c>(scraper);

    autonSchedule.enqueue<movepoint_c>(point{ 55.5, -53.5 },
                                       chassis,
                                       mx,
                                       my,
                                       poseCFG{ .reversed = false,
                                                .minSpeed = 100,
                                                .maxSpeed = 127,
                                                .timeout = 1000 });

    autonSchedule.enqueue<turnheading_c>(
        225, chassis, mx, my, poseCFG{ .minSpeed = 80, .timeout = 1200 });
    autonSchedule.enqueue<turnheading_c>(
        225, chassis, mx, my, poseCFG{ .minSpeed = 0, .timeout = 200 });
    autonSchedule.enqueue<movepoint_c>(point{ 26.5, -49 },
                                       chassis,
                                       mx,
                                       my,
                                       poseCFG{ .reversed = true,
                                                .minSpeed = 40,
                                                .maxSpeed = 127,
                                                .timeout = 500 });
    autonSchedule.enqueue<mech_state_c>(OUT_MIDDLE, outtake);

    autonSchedule.enqueue<movepoint_c>(
        point{ -999, 999 },
        chassis,
        mx,
        my,
        poseCFG{
            .reversed = true, .minSpeed = 40, .maxSpeed = 50, .timeout = 750 });
    autonSchedule.enqueue<setPose_c>(
        lemlib::Pose{ mx * float(40_cm),
                      my * float(-40_cm),
                      adjustHeading(225, mx, my) }); // y:38
    autonSchedule.enqueue<mech_state_c>(OUT_HOARD, outtake);

    autonSchedule.enqueue<movepoint_c>(point{ 100, -96 },
                                       chassis,
                                       mx,
                                       my,
                                       poseCFG{ .reversed = false,
                                                .minSpeed = 127,
                                                .maxSpeed = 127,
                                                .timeout = 1000 });

    autonSchedule.enqueue<movepose_c>(lemlib::Pose{ 120, -108, 270 },
                                      chassis,
                                      mx,
                                      my,
                                      poseCFG{ .reversed = false,
                                               .minSpeed = 80,
                                               .maxSpeed = 127,
                                               .timeout = 400 });
    autonSchedule.enqueue<turnheading_c>(
        270, chassis, mx, my, poseCFG{ .minSpeed = 90, .timeout = 1200 });
    autonSchedule.enqueue<movepose_c>(
        lemlib::Pose{ 190, -122, 270 },
        chassis,
        mx,
        my,
        poseCFG{ .reversed = false,
                 .minSpeed = 80,
                 .maxSpeed = 127,
                 .timeout = doSkills ? 3000 : 1250 });
    autonSchedule.enqueue<movepose_c>(lemlib::Pose{ 70, -123, 270 },
                                      chassis,
                                      mx,
                                      my,
                                      poseCFG{ .reversed = true,
                                               .minSpeed = 80,
                                               .maxSpeed = 127,
                                               .timeout = 780 });
    autonSchedule.enqueue<mech_state_c>(OUT_TOP, outtake);
    autonSchedule.enqueue<mech_state_c>(OUT_TOP, outtake);

    if (doSkills) {
        autonSchedule.enqueue<movepose_c>(
            lemlib::Pose{ 0, -122, 270 },
            chassis,
            mx,
            my,
            poseCFG{ .reversed = false,
                     .minSpeed = 40,
                     .maxSpeed = 60,
                     .timeout = doSkills ? 3000 : 1250 });
        autonSchedule.enqueue<movepose_c>(lemlib::Pose{ 999, -122, 270 },
                                          chassis,
                                          mx,
                                          my,
                                          poseCFG{ .reversed = false,
                                                   .minSpeed = 127,
                                                   .maxSpeed = 127,
                                                   .timeout = 1000 });
        autonSchedule.enqueue<movepose_c>(lemlib::Pose{ 180, 999, 0 },
                                          chassis,
                                          mx,
                                          my,
                                          poseCFG{ .reversed = false,
                                                   .minSpeed = 127,
                                                   .maxSpeed = 127,
                                                   .timeout = 5000 });
    }
    // autonSchedule.enqueue<mech_state_c>(OUT_MIDDLE, outtake);
    // autonSchedule.enqueue<movepoint_c>(
    //     point{ 36, -36 },
    //     chassis,
    //     mx,
    //     my,
    //     poseCFG{ .reversed = true, .minSpeed = 30, .maxSpeed = 105 });

    // autonSchedule.enqueue<movepoint_c>(
    //     point{ 36, 36 },
    //     chassis,
    //     mx,
    //     my,
    //     poseCFG{ .minSpeed = 85, .maxSpeed = 105 });
    // autonSchedule.enqueue<movepoint_c>(
    //     point{ 30, 30 },
    //     chassis,
    //     mx,
    //     my,
    //     poseCFG{ .minSpeed = 40, .maxSpeed = 100 });

    /*autonSchedule.enqueue<movepoint_c>(
        point{ 131, 104 },
        chassis,
        mx,
        my,
        poseCFG{ .minSpeed = 100, .timeout = 200, .earlyExitRange = 4 });
    autonSchedule.enqueue<togglePneu_c>(scraper);
    autonSchedule.enqueue<movepoint_c>(
        point{ 118, 113 }, chassis, mx, my, poseCFG{ .minSpeed = 75 });

    // autonSchedule.enqueue<movepose_c>(
    //     lemlib::Pose{ 130, 120, 270 }, chassis, mx, my, poseCFG{});
    // autonSchedule.enqueue<movepose_c>(lemlib::Pose{ 135, 120, 270 },
    //                                   chassis,
    //                                   mx,
    //                                   my,
    //                                   poseCFG{ .earlyExitRange = 2 });

    autonSchedule.enqueue<turnheading_c>(
        270, chassis, mx, my, poseCFG{ .timeout = 300 });
    // autonSchedule.enqueue<setPose_c>(lemlib::Pose{
    //     mx * float(120_cm), my * 121_cm, adjustHeading(9999, mx, my) });
    autonSchedule.enqueue<movepoint_c>(
        point{ 180, 120 },
        chassis,
        mx,
        my,
        poseCFG{ .minSpeed = 110, .maxSpeed = 116, .timeout = 750 });
    autonSchedule.enqueue<movepoint_c>(
        point{ 999, 120 },
        chassis,
        mx,
        my,
        poseCFG{ .minSpeed = 40,
                 .maxSpeed = 60,
                 .timeout = (doSkills ? 2000 : 75) });
    autonSchedule.enqueue<movepose_c>(
        lemlib::Pose{ 100, 124, 270 },
        chassis,
        mx,
        my,
        poseCFG{ .reversed = true, .minSpeed = 85, .timeout = 900 });

    autonSchedule.enqueue<movepose_c>(
        lemlib::Pose{ 70, 126.5, 270 },
        chassis,
        mx,
        my,
        poseCFG{ .reversed = true, .minSpeed = 25, .timeout = 900 });

    autonSchedule.enqueue<mech_state_c>(OUT_TOP, outtake);
    autonSchedule.enqueue<movepose_c>(
        lemlib::Pose{ 0, 126.5, 270 },
        chassis,
        mx,
        my,
        poseCFG{ .reversed = true,
                 .minSpeed = 20,
                 .maxSpeed = 40,
                 .timeout = (doSkills ? 3500 : 1850) });
    autonSchedule.enqueue<mech_state_c>(OUT_HOARD, outtake);

    autonSchedule.enqueue<togglePneu_c>(scraper);
    autonSchedule.enqueue<movepoint_c>(
        point{ 999, 125 },
        chassis,
        mx,
        my,
        poseCFG{ .minSpeed = 127, .maxSpeed = 127, .timeout = 150 });

    autonSchedule.enqueue<movepose_c>(lemlib::Pose{ 100, -20, 180 },
                                      chassis,
                                      mx,
                                      my,
                                      poseCFG{ .minSpeed = 127 });
    autonSchedule.enqueue<mech_state_c>(OUT_HOARD, outtake);

    autonSchedule.enqueue<movepoint_c>(
        point{ 77, -57 },
        chassis,
        mx,
        my,
        poseCFG{ .minSpeed = 127, .timeout = 680 });
    // SCRAPER TOGGLES HERE ----------------
    autonSchedule.enqueue<togglePneu_c>(scraper);
    //  autonSchedule.enqueue<movepoint_c>(
    //      point{ 73, -61 }, chassis, mx, my, poseCFG{ .minSpeed = 100 });
    autonSchedule.enqueue<turnheading_c>(
        225, chassis, mx, my, poseCFG{ .minSpeed = 50, .timeout = 800 });
    // autonSchedule.enqueue<movepose_c>(
    //     lemlib::Pose{ 60, -60, 225 },
    //     chassis,
    //     mx,
    //     my,
    //     poseCFG{ .reversed = true, .minSpeed = 80 });

    // autonSchedule.enqueue<movepoint_c>(
    //     point{ 45, -50 },
    //     chassis,
    //     mx,
    //     my,
    //     poseCFG{ .reversed = true, .minSpeed = 80, .timeout = 300 });

    autonSchedule.enqueue<movepose_c>(
        lemlib::Pose{ 18, -18, 225 },
        chassis,
        mx,
        my,
        poseCFG{ .reversed = true, .minSpeed = 20, .timeout = 900 });
    autonSchedule.enqueue<mech_state_c>(OUT_MIDDLE, outtake);
    autonSchedule.enqueue<movepose_c>(
        lemlib::Pose{ 18, -18, 225 },
        chassis,
        mx,
        my,
        poseCFG{ .reversed = true, .minSpeed = 20, .timeout = 310 });
    autonSchedule.enqueue<turnheading_c>(
        225, chassis, mx, my, poseCFG{ .minSpeed = 40, .timeout = 130 });
    autonSchedule.enqueue<wait_c>(doSkills ? 1500 : 300);
    // autonSchedule.enqueue<movepose_c>(
    //     lemlib::Pose{ 0, 0, 225 },
    //     chassis,
    //     mx,
    //     my,
    //     poseCFG{
    //         .reversed = true, .minSpeed = 20, .maxSpeed = 40, .timeout =
    700
    //         });
    autonSchedule.enqueue<setPose_c>(
        lemlib::Pose{ mx * float(31.5_cm), my * float(-31.5_cm), 999999 });
    // autonSchedule.enqueue<togglePneu_c>(scraper);
    autonSchedule.enqueue<mech_state_c>(OUT_HOARD, outtake);

    autonSchedule.enqueue<movepoint_c>(
        point{ 100, -103 },
        chassis,
        mx,
        my,
        poseCFG{ .minSpeed = 120, .timeout = 450 });
    // autonSchedule.enqueue<togglePneu_c>(scraper);
    autonSchedule.enqueue<movepoint_c>(
        point{ 102, -103 }, chassis, mx, my, poseCFG{ .minSpeed = 60 });
    autonSchedule.enqueue<turnheading_c>(
        270, chassis, mx, my, poseCFG{ .minSpeed = 40, .timeout = 400 });
    // autonSchedule.enqueue<togglePneu_c>(scraper);
    autonSchedule.enqueue<mech_state_c>(OUT_HOARD, outtake);
    autonSchedule.enqueue<movepoint_c>(
        point{ 190, -120 },
        chassis,
        mx,
        my,
        poseCFG{ .minSpeed = 110, .timeout = (doSkills ? 1200 : 525) });
    autonSchedule.enqueue<movepoint_c>(
        point{ 200, -120 },
        chassis,
        mx,
        my,
        poseCFG{ .maxSpeed = 110, .timeout = 400 });
    autonSchedule.enqueue<movepose_c>(lemlib::Pose{ 70, -121, 270 },
                                      chassis,
                                      mx,
                                      my,
                                      poseCFG{ .reversed = true,
                                               .minSpeed = 90,
                                               .maxSpeed = 127,
                                               .timeout = 800 });
    autonSchedule.enqueue<mech_state_c>(OUT_TOP, outtake);
    autonSchedule.enqueue<movepose_c>(
        lemlib::Pose{ 0, -121, 270 },
        chassis,
        mx,
        my,
        poseCFG{ .reversed = true,
                 .minSpeed = 40,
                 .maxSpeed = 60,
                 .timeout = (doSkills ? 3500 : 1000) });
    autonSchedule.enqueue<outtake_toggle_c>(false, outtake);
    if (doSkills) {
        autonSchedule.enqueue<movepose_c>(lemlib::Pose{ 999, -117.5, 270 },
                                          chassis,
                                          mx,
                                          my,
                                          poseCFG{ .reversed = false,
                                                   .minSpeed = 120,
                                                   .maxSpeed = 120,
                                                   .timeout = 300 });
        autonSchedule.enqueue<movepoint_c>(point{ 175, 9999 },
                                           chassis,
                                           mx,
                                           my,
                                           poseCFG{ .reversed = false,
                                                    .minSpeed = 120,
                                                    .maxSpeed = 120,
                                                    .timeout = 5000 });
    }

    //  autonSchedule.enqueue<movepose_c>(lemlib::Pose{ 70, -120, 270 },
    //                                      chassis,
    //                                      mx,
    //                                      my,
    //                                      poseCFG{ .reversed = true });
    // autonSchedule.enqueue<mech_state_c>(OUT_TOP, outtake);
    // autonSchedule.enqueue<wait_c>(1000);
    // autonSchedule.enqueue<outtake_toggle_c>(false, outtake);
    */
    while (autonSchedule.tick()) {
        outtake.loop();
        pros::delay(10);
        // std::cout << chassis.getPose().x << std::endl;
    }
    std::cout << double(pros::millis() - iteration) / 1000 << std::endl;
    std::cout << chassis.getPose().x * 2.54 << std::endl;
    std::cout << chassis.getPose().y * 2.54 << std::endl;
}

void park() {
    autonSchedule.enqueue<setPose_c>(lemlib::Pose{ 0_cm, 0_cm, 0 });
    autonSchedule.enqueue<mech_state_c>(OUT_TOP, outtake);
    autonSchedule.enqueue<move_pose_c>(
        lemlib::Pose{ 0, -30, 0 },
        chassis,
        poseCFG{ .reversed = true, .minSpeed = 90, .timeout = 500 });
    autonSchedule.enqueue<move_pose_c>(
        lemlib::Pose{ 0, 3000, 0 },
        chassis,
        poseCFG{ .minSpeed = 127, .timeout = 400 });
    autonSchedule.enqueue<togglePneu_c>(scraper);
    autonSchedule.enqueue<move_pose_c>(
        lemlib::Pose{ 0, 3000, 0 },
        chassis,
        poseCFG{ .minSpeed = 127, .timeout = 1350 });
    autonSchedule.enqueue<togglePneu_c>(scraper);

    while (autonSchedule.tick()) {
        outtake.loop();
        pros::delay(10);
        // std::cout << chassis.getPose().x << std::endl;
    }
}

void testing_pid() {
    chassis.setPose(0, 0, 0);
    // chassis.moveToPose(0, 24, 0, 9999);
    std::cout << chassis.getPose().x << std::endl;
    std::cout << chassis.getPose().y << std::endl;
    chassis.turnToHeading(90, 99999);
}
void auto_bottomGoal(bool isBlue) {
    float mx = isBlue ? 1 : -1;
    float my = isBlue ? 1 : -1;
    int iteration = pros::millis();
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    // chassis.setPose(mx * 124_cm, my * -48_cm, adjustHeading(90, mx, my));
    autonSchedule.enqueue<setPose_c>(
        lemlib::Pose{ mx * float(119.38_cm),
                      my * float(36.28_cm),
                      adjustHeading(90, mx, my) }); // y:38
    autonSchedule.enqueue<togglePneu_c>(descorer_l);

    autonSchedule.enqueue<movepose_c>(
        lemlib::Pose{ 53, 56, 90 },
        chassis,
        mx,
        my,
        poseCFG{ .minSpeed = 105, .timeout = 700, .earlyExitRange = 3 });
    autonSchedule.enqueue<mech_state_c>(OUT_HOARD, outtake);
    autonSchedule.enqueue<togglePneu_c>(scraper);
    autonSchedule.enqueue<movepoint_c>(
        point{ 60, 56 },
        chassis,
        mx,
        my,
        poseCFG{ .minSpeed = 40, .maxSpeed = 60, .timeout = 300 });
    // autonSchedule.enqueue<movepoint_c>(point{ 64, 56 },
    //                                    chassis,
    //                                    mx,
    //                                    my,
    //                                    poseCFG{ .reversed = true,
    //                                             .minSpeed = 85,
    //                                             .maxSpeed = 105,
    //                                             .earlyExitRange = 1 });
    // MIDGOAL_START
    // autonSchedule.enqueue<turnheading_c>(
    //     135, chassis, mx, my, poseCFG{ .minSpeed = 20, .timeout = 500 });
    autonSchedule.enqueue<togglePneu_c>(scraper);

    // autonSchedule.enqueue<movepoint_c>(
    //     point{ 46, 42 },
    //     chassis,
    //     mx,
    //     my,
    //     poseCFG{ .minSpeed = 85, .maxSpeed = 105, .earlyExitRange = 1 });
    // autonSchedule.enqueue<movepose_c>(
    //     lemlib::Pose{ 39, 35, 135 },
    //     chassis,
    //     mx,
    //     my,
    //     poseCFG{ .minSpeed = 20, .maxSpeed = 105, .timeout = 500 });

    // autonSchedule.enqueue<mech_state_c>(OUT_BOTTOM, outtake);
    // autonSchedule.enqueue<wait_c>(400);
    // autonSchedule.enqueue<mech_state_c>(OUT_HOARD, outtake);

    // autonSchedule.enqueue<movepose_c>(
    //     lemlib::Pose{ 32, 32, 135 },
    //     chassis,
    //     mx,
    //     my,
    //     poseCFG{ .minSpeed = 40, .maxSpeed = 105, .timeout = 200 });
    // BOTTOM_END
    autonSchedule.enqueue<movepoint_c>(
        point{ 80, 80 },
        chassis,
        mx,
        my,
        poseCFG{ .reversed = true, .minSpeed = 127, .maxSpeed = 127 });
    autonSchedule.enqueue<movepoint_c>(
        point{ 95, 97 },
        chassis,
        mx,
        my,
        poseCFG{ .reversed = true, .minSpeed = 30, .maxSpeed = 127 });
    autonSchedule.enqueue<togglePneu_c>(scraper);

    autonSchedule.enqueue<turnheading_c>(
        270, chassis, mx, my, poseCFG{ .minSpeed = 20 });

    autonSchedule.enqueue<movepoint_c>(
        point{ 180, 121 },
        chassis,
        mx,
        my,
        poseCFG{ .minSpeed = 65, .maxSpeed = 75, .timeout = 250 });
    autonSchedule.enqueue<movepoint_c>(
        point{ 180, 121 },
        chassis,
        mx,
        my,
        poseCFG{ .minSpeed = 75, .maxSpeed = 75, .timeout = 1000 });
    autonSchedule.enqueue<movepoint_c>(point{ 84, 121 },
                                       chassis,
                                       mx,
                                       my,
                                       poseCFG{ .reversed = true,
                                                .minSpeed = 127,
                                                .maxSpeed = 127,
                                                .timeout = 600 });
    autonSchedule.enqueue<mech_state_c>(OUT_TOP, outtake);

    autonSchedule.enqueue<movepoint_c>(
        point{ 0, 123 },
        chassis,
        mx,
        my,
        poseCFG{
            .reversed = true, .minSpeed = 40, .maxSpeed = 60, .timeout = 400 });
    autonSchedule.enqueue<mech_state_c>(OUT_BOTTOM, outtake);
    autonSchedule.enqueue<togglePneu_c>(scraper);
    autonSchedule.enqueue<movepoint_c>(
        point{ 0, 123 },
        chassis,
        mx,
        my,
        poseCFG{
            .reversed = true, .minSpeed = 40, .maxSpeed = 60, .timeout = 250 });
    autonSchedule.enqueue<mech_state_c>(OUT_TOP, outtake);

    autonSchedule.enqueue<movepoint_c>(point{ 0, 123 },
                                       chassis,
                                       mx,
                                       my,
                                       poseCFG{ .reversed = true,
                                                .minSpeed = 40,
                                                .maxSpeed = 60,
                                                .timeout = 1250 });
    autonSchedule.enqueue<mech_state_c>(OUT_BOTTOM, outtake);
    autonSchedule.enqueue<movepoint_c>(
        point{ 0, 123 },
        chassis,
        mx,
        my,
        poseCFG{
            .reversed = true, .minSpeed = 40, .maxSpeed = 60, .timeout = 250 });
    autonSchedule.enqueue<mech_state_c>(OUT_TOP, outtake);

    autonSchedule.enqueue<movepoint_c>(point{ 0, 124 },
                                       chassis,
                                       mx,
                                       my,
                                       poseCFG{ .reversed = true,
                                                .minSpeed = 40,
                                                .maxSpeed = 60,
                                                .timeout = 1250 });

    autonSchedule.enqueue<mech_state_c>(OUT_HOARD, outtake);

    // autonSchedule.enqueue<togglePneu_c>(scraper);
    autonSchedule.enqueue<movepoint_c>(
        point{ 999, 125 },
        chassis,
        mx,
        my,
        poseCFG{ .minSpeed = 127, .maxSpeed = 127, .timeout = 150 });
    autonSchedule.enqueue<movepose_c>(
        lemlib::Pose{ 135, 158, 270 },
        chassis,
        mx,
        my,
        poseCFG{ .minSpeed = 120, .maxSpeed = 120, .timeout = 1000 });
    autonSchedule.enqueue<turnheading_c>(
        270, chassis, mx, my, poseCFG{ .minSpeed = 40, .timeout = 400 });
    autonSchedule.enqueue<togglePneu_c>(descorer_l);

    autonSchedule.enqueue<movepoint_c>(point{ 60, 155 },
                                       chassis,
                                       mx,
                                       my,
                                       poseCFG{ .reversed = true,
                                                .minSpeed = 80,
                                                .maxSpeed = 80,
                                                .timeout = 1000 });
    autonSchedule.enqueue<swing_heading_c>(
        340,
        lemlib::DriveSide::LEFT,
        chassis,
        mx,
        my,
        swingCFG{
            .maxSpeed = 120,
            .minSpeed = 80,
            .timeout = 8000,
            .dirPath = lemlib::AngularDirection::CW_CLOCKWISE,

        });

    while (autonSchedule.tick()) {
        outtake.loop();
        pros::delay(10);
        // std::cout << chassis.getPose().x << std::endl;
    }
}

void auton_skills() {
    float mx = 1;
    float my = 1;
    int iteration = pros::millis();
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    // chassis.setPose(mx * 124_cm, my * -48_cm, adjustHeading(90, mx, my));
    autonSchedule.enqueue<setPose_c>(lemlib::Pose{
        mx * float(-160_cm), my * float(-40_cm), adjustHeading(0, mx, my) });
    autonSchedule.enqueue<movepose_c>(
        lemlib::Pose{ -160, -60, 0 },
        chassis,
        mx,
        my,
        poseCFG{ .reversed = true, .minSpeed = 105, .timeout = 300 });
    autonSchedule.enqueue<mech_state_c>(OUT_HOARD, outtake);
    autonSchedule.enqueue<movepose_c>(
        lemlib::Pose{ -160, 45, 0 },
        chassis,
        mx,
        my,
        poseCFG{ .reversed = false, .minSpeed = 105, .timeout = 2000 });
    autonSchedule.enqueue<movepose_c>(
        lemlib::Pose{ -119, 101, 100 },
        chassis,
        mx,
        my,
        poseCFG{ .reversed = false, .minSpeed = 105, .timeout = 1500 });
    autonSchedule.enqueue<movepose_c>(
        lemlib::Pose{ -80, 80, 135 },
        chassis,
        mx,
        my,
        poseCFG{ .reversed = false, .minSpeed = 60, .timeout = 1500 });
    autonSchedule.enqueue<turnheading_c>(
        315, chassis, mx, my, poseCFG{ .minSpeed = 90, .timeout = 1200 });
    autonSchedule.enqueue<movepose_c>(
        lemlib::Pose{ -50, 50, 315 },
        chassis,
        mx,
        my,
        poseCFG{ .reversed = true, .minSpeed = 60, .timeout = 1500 });
    autonSchedule.enqueue<movepose_c>(
        lemlib::Pose{ -40, 40, 135 },
        chassis,
        mx,
        my,
        poseCFG{ .reversed = true, .minSpeed = 20, .timeout = 1500 });
    autonSchedule.enqueue<wait_c>(1000);
    autonSchedule.enqueue<movepoint_c>(
        point{ -100, 100 },
        chassis,
        mx,
        my,
        poseCFG{ .reversed = false, .minSpeed = 120, .timeout = 1500 });
    autonSchedule.enqueue<movepose_c>(
        lemlib::Pose{ -130, 120, 270 },
        chassis,
        mx,
        my,
        poseCFG{ .reversed = false, .minSpeed = 40, .timeout = 1500 });
    autonSchedule.enqueue<togglePneu_c>(scraper);
    autonSchedule.enqueue<movepose_c>(
        lemlib::Pose{ -180, 120, 270 },
        chassis,
        mx,
        my,
        poseCFG{ .reversed = false, .minSpeed = 80, .timeout = 1500 });
    autonSchedule.enqueue<movepose_c>(
        lemlib::Pose{ -120, 120, 270 },
        chassis,
        mx,
        my,
        poseCFG{ .reversed = true, .minSpeed = 80, .timeout = 1500 });
    // LONGGOAL1
    autonSchedule.enqueue<togglePneu_c>(scraper);
    autonSchedule.enqueue<swing_heading_c>(
        90,
        lemlib::DriveSide::RIGHT,
        chassis,
        mx,
        my,
        swingCFG{ .minSpeed = 90, .timeout = 1200 });
    autonSchedule.enqueue<movepose_c>(
        lemlib::Pose{ 90, 50, 90 },
        chassis,
        mx,
        my,
        poseCFG{ .minSpeed = 105, .timeout = 1000, .earlyExitRange = 3 });
    autonSchedule.enqueue<togglePneu_c>(scraper);

    autonSchedule.enqueue<movepose_c>(
        lemlib::Pose{ 130, 120, 90 },
        chassis,
        mx,
        my,
        poseCFG{ .minSpeed = 60, .timeout = 1200 });
    autonSchedule.enqueue<movepose_c>(
        lemlib::Pose{ 180, 120, 90 },
        chassis,
        mx,
        my,
        poseCFG{ .minSpeed = 105, .timeout = 700, .earlyExitRange = 3 });
    autonSchedule.enqueue<wait_c>(1000);
    // LONGGOAL 2 Start
}

void codingBlocks() {
    float mx = -1;
    float my = -1;

    autonSchedule.enqueue<setPose_c>(
        lemlib::Pose{ mx * float(160_cm),
                      my * float(40_cm),
                      adjustHeading(0, mx, my) }); // y:38
    autonSchedule.enqueue<togglePneu_c>(scraper);
    autonSchedule.enqueue<turnheading_c>(
        270, chassis, mx, my, poseCFG{ .minSpeed = 90, .timeout = 1200 });
    autonSchedule.enqueue<movepose_c>(
        lemlib::Pose{ 53, 56, 90 },
        chassis,
        mx,
        my,
        poseCFG{ .minSpeed = 105, .timeout = 700, .earlyExitRange = 3 });
    autonSchedule.enqueue<mech_state_c>(OUT_HOARD, outtake);
    autonSchedule.enqueue<togglePneu_c>(scraper);
    autonSchedule.enqueue<movepoint_c>(
        point{ 60, 56 },
        chassis,
        mx,
        my,
        poseCFG{ .minSpeed = 40, .maxSpeed = 60, .timeout = 300 });
    autonSchedule.enqueue<wait_c>(1000);
}

void halfAuto(bool isBlue) {
    float mx = isBlue ? 1 : -1;
    float my = isBlue ? -1 : 1;
    int iteration = pros::millis();
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    autonSchedule.enqueue<setPose_c>(
        lemlib::Pose{ mx * float(119.38_cm),
                      my * float(36.28_cm),
                      adjustHeading(90, mx, my) }); // y:38
    autonSchedule.enqueue<togglePneu_c>(descorer_l);

    autonSchedule.enqueue<mech_state_c>(OUT_HOARD, outtake);

    autonSchedule.enqueue<movepoint_c>(
        point{ 31, 66 },
        chassis,
        mx,
        my,
        poseCFG{ .minSpeed = 80, .maxSpeed = 127, .timeout = 1000 });
    autonSchedule.enqueue<movepoint_c>(
        point{ -20, 104 },
        chassis,
        mx,
        my,
        poseCFG{ .minSpeed = 80, .maxSpeed = 127, .timeout = 1000 });
    autonSchedule.enqueue<togglePneu_c>(scraper);
}