#include "autons.hpp"
#include "commandHandling.hpp"
#include "consts.hpp"
#include "lemlib/asset.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/pose.hpp"
#include "main.h"
#include "pros/distance.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rtos.h"
#include "pros/rtos.hpp"
#include "zcommands/chassisCommands.hpp"
#include "zcommands/mechCommands.hpp"
#include "zcommands/sensorCommands.hpp"
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
}

void auton_skills() {
    float mx = 1;
    float my = 1;
    int iteration = pros::millis();

    pros::Distance dist(11);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    static std::array<pros::Distance, 4> distance = { pros::Distance(11),
                                                      pros::Distance(13),
                                                      pros::Distance(6),
                                                      pros::Distance(1) };

    static std::vector<lemlib::Pose> offsets{
        lemlib::Pose{ 3, 7.125, 0 },
        lemlib::Pose(-13.5_cm, 11.45_cm, -90),
        lemlib::Pose{ 11.5_cm, -12_cm, 180 },
        lemlib::Pose(13.5_cm, 11.45_cm, 90),
    };
    autonSchedule.enqueue<togglePneu_c>(descorer_l);

    // chassis.setPose(mx * 124_cm, my * -48_cm, adjustHeading(90, mx, my));
    autonSchedule.enqueue<setPose_c>(
        lemlib::Pose{ mx * float(-160_cm), my * float(-37.6_cm), 180 });
    autonSchedule.enqueue<movepose_c>(lemlib::Pose{ -160, -121.5, 180 },
                                      chassis,
                                      mx,
                                      my,
                                      poseCFG{ .reversed = false,
                                               .minSpeed = 0,
                                               .maxSpeed = 100,
                                               .timeout = 1500 });
    autonSchedule.enqueue<togglePneu_c>(scraper);

    autonSchedule.enqueue<turnheading_c>(
        90, chassis, mx, my, poseCFG{ .minSpeed = 0, .timeout = 1200 });

    autonSchedule.enqueue<movepoint_c>(
        point{ -184, -121.5 },
        chassis,
        mx,
        my,
        poseCFG{ .minSpeed = 40, .maxSpeed = 60, .timeout = 1000 });
    autonSchedule.enqueue<mech_state_c>(OUT_HOARD, outtake);
    autonSchedule.enqueue<movepoint_c>(
        point{ -160, -123 },
        chassis,
        mx,
        my,
        poseCFG{
            .reversed = true, .minSpeed = 60, .maxSpeed = 60, .timeout = 100 });
    autonSchedule.enqueue<movepoint_c>(
        point{ -200, -123 },
        chassis,
        mx,
        my,
        poseCFG{ .minSpeed = 20, .maxSpeed = 30, .timeout = 1500 });

    // autonSchedule.enqueue<movepoint_c>(
    //     point{ -200, -120 },
    //     chassis,
    //     mx,
    //     my,
    //     poseCFG{ .minSpeed = 10, .maxSpeed = 30, .timeout = 2000 });

    autonSchedule.enqueue<movepose_c>(lemlib::Pose{ -120, -155, 90 },
                                      chassis,
                                      mx,
                                      my,
                                      poseCFG{ .reversed = true,
                                               .minSpeed = 0,
                                               .maxSpeed = 127,
                                               .timeout = 2000,
                                               .earlyExitRange = 3 });
    autonSchedule.enqueue<outtake_toggle_c>(false, outtake);

    autonSchedule.enqueue<movepose_c>(lemlib::Pose{ 100, -153, 90 },
                                      chassis,
                                      mx,
                                      my,
                                      poseCFG{ .reversed = true,
                                               .minSpeed = 0,
                                               .maxSpeed = 127,
                                               .timeout = 2000 });
    autonSchedule.enqueue<swing_heading_c>(
        270,
        lemlib::DriveSide::RIGHT,
        chassis,
        mx,
        my,
        swingCFG{
            .maxSpeed = 120,
            .minSpeed = 0,
            .timeout = 8000,
            .dirPath = lemlib::AngularDirection::CW_CLOCKWISE,

        });
    autonSchedule.enqueue<movepose_c>(lemlib::Pose{ 50, -123, 270 },
                                      chassis,
                                      mx,
                                      my,
                                      poseCFG{ .reversed = true,
                                               .minSpeed = 0,
                                               .maxSpeed = 100,
                                               .timeout = 1500 });
    autonSchedule.enqueue<outtake_toggle_c>(true, outtake);

    autonSchedule.enqueue<mech_state_c>(OUT_TOP, outtake);

    autonSchedule.enqueue<movepose_c>(
        lemlib::Pose{ 0, -123, 270 },
        chassis,
        mx,
        my,
        poseCFG{
            .reversed = true, .minSpeed = 0, .maxSpeed = 40, .timeout = 2000 });
    autonSchedule.enqueue<setPose_c>(
        lemlib::Pose{ float(73_cm), float(-122_cm), 999999 });
    autonSchedule.enqueue<mech_state_c>(OUT_HOARD, outtake);

    autonSchedule.enqueue<movepoint_c>(point{ 120, -118 },
                                       chassis,
                                       mx,
                                       my,
                                       poseCFG{ .reversed = false,
                                                .minSpeed = 100,
                                                .maxSpeed = 127,
                                                .timeout = 2000 });
    autonSchedule.enqueue<movepose_c>(lemlib::Pose{ 180, -118, 270 },
                                      chassis,
                                      mx,
                                      my,
                                      poseCFG{ .reversed = false,
                                               .minSpeed = 60,
                                               .maxSpeed = 70,
                                               .timeout = 1500 });
    autonSchedule.enqueue<movepose_c>(lemlib::Pose{ 250, -118, 270 },
                                      chassis,
                                      mx,
                                      my,
                                      poseCFG{ .reversed = false,
                                               .minSpeed = 30,
                                               .maxSpeed = 40,
                                               .timeout = 1000 });
    autonSchedule.enqueue<movepose_c>(lemlib::Pose{ 70, -119, 270 },
                                      chassis,
                                      mx,
                                      my,
                                      poseCFG{ .reversed = true,
                                               .minSpeed = 60,
                                               .maxSpeed = 80,
                                               .timeout = 2000 });
    autonSchedule.enqueue<wait_c>(250);

    autonSchedule.enqueue<mech_state_c>(OUT_TOP, outtake);
    autonSchedule.enqueue<movepose_c>(lemlib::Pose{ 0, -119, 270 },
                                      chassis,
                                      mx,
                                      my,
                                      poseCFG{ .reversed = true,
                                               .minSpeed = 30,
                                               .maxSpeed = 40,
                                               .timeout = 2500 });
    // BAP ZE LONGGOAL
    // autonSchedule.enqueue<movepose_c>(lemlib::Pose{ 200, -119, 270 },
    //                                   chassis,
    //                                   mx,
    //                                   my,
    //                                   poseCFG{ .reversed = false,
    //                                            .minSpeed = 75,
    //                                            .maxSpeed = 80,
    //                                            .timeout = 200 });
    // autonSchedule.enqueue<movepose_c>(lemlib::Pose{ 0, -119, 270 },
    //                                   chassis,
    //                                   mx,
    //                                   my,
    //                                   poseCFG{ .reversed = true,
    //                                            .minSpeed = 100,
    //                                            .maxSpeed = 120,
    //                                            .timeout = 500 });
    autonSchedule.enqueue<movepose_c>(
        lemlib::Pose{ 138, -80, 0 },
        chassis,
        mx,
        my,
        poseCFG{ .reversed = false, .minSpeed = 10, .timeout = 2500 });
    autonSchedule.enqueue<turnheading_c>(
        0, chassis, mx, my, poseCFG{ .minSpeed = 10, .timeout = 1200 });
    autonSchedule.enqueue<mech_state_c>(OUT_HOARD, outtake);

    autonSchedule.enqueue<togglePneu_c>(scraper);

    autonSchedule.enqueue<movepose_c>(
        lemlib::Pose{ 160, -40, 0 },
        chassis,
        mx,
        my,
        poseCFG{ .reversed = false, .timeout = 2500 });
    autonSchedule.enqueue<movepose_c>(
        lemlib::Pose{ 160, -60, 0 },
        chassis,
        mx,
        my,
        poseCFG{ .reversed = true, .timeout = 250 });
    // PARKZONE
    autonSchedule.enqueue<movepose_c>(
        lemlib::Pose{ 159, 999, 0 },
        chassis,
        mx,
        my,
        poseCFG{ .reversed = false, .minSpeed = 127, .timeout = 2200 });
    autonSchedule.enqueue<togglePneu_c>(scraper);

    autonSchedule.enqueue<movepose_c>(
        lemlib::Pose{ 159, -999, 0 },
        chassis,
        mx,
        my,
        poseCFG{ .reversed = true, .maxSpeed = 65, .timeout = 1000 });
    autonSchedule.enqueue<setPose_c>(lemlib::Pose{ mx * float(159.385_cm),
                                                   my * float(43.18_cm),
                                                   adjustHeading(0, mx, my) });
    autonSchedule.enqueue<movepoint_c>(
        point{ 120, 120 },
        chassis,
        mx,
        my,
        poseCFG{ .minSpeed = 0, .maxSpeed = 127, .timeout = 1000 });
    autonSchedule.enqueue<turnheading_c>(
        270, chassis, mx, my, poseCFG{ .minSpeed = 0, .timeout = 1000 });
    autonSchedule.enqueue<movepoint_c>(
        point{ 180, 120 },
        chassis,
        mx,
        my,
        poseCFG{ .minSpeed = 0, .maxSpeed = 80, .timeout = 750 });
    autonSchedule.enqueue<movepoint_c>(
        point{ 120, 120 },
        chassis,
        mx,
        my,
        poseCFG{ .minSpeed = 0, .maxSpeed = 120, .timeout = 1000 });
    autonSchedule.enqueue<movepoint_c>(
        point{ 60, -60 },
        chassis,
        mx,
        my,
        poseCFG{ .minSpeed = 0, .maxSpeed = 127, .timeout = 2000 });

    while (autonSchedule.tick()) {
        outtake.loop();
        pros::delay(10);
        // std::cout << chassis.getPose().x << std::endl;
    }
    std::cout << double(pros::millis() - iteration) / 1000 << std::endl;
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
        lemlib::Pose{ mx * float(118.11_cm),
                      my * float(18.415_cm),
                      adjustHeading(90, mx, my) }); // y:38
    autonSchedule.enqueue<togglePneu_c>(descorer_l);

    autonSchedule.enqueue<mech_state_c>(OUT_HOARD, outtake);
    autonSchedule.enqueue<swing_heading_c>(
        55,
        lemlib::DriveSide::LEFT,
        chassis,
        mx,
        my,
        swingCFG{ .minSpeed = 40,
                  .earlyExitRange = 3,
                  .dirPath = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE

        });

    autonSchedule.enqueue<movepoint_c>(point{ 35, 96 },
                                       chassis,
                                       mx,
                                       my,
                                       poseCFG{ .minSpeed = 30,
                                                .maxSpeed = 127,

                                                .timeout = 2000,
                                                .earlyExitRange = 7 });
    autonSchedule.enqueue<togglePneu_c>(scraper);
    autonSchedule.enqueue<movepoint_c>(point{ 32, 98 },
                                       chassis,
                                       mx,
                                       my,
                                       poseCFG{ .minSpeed = 30,
                                                .maxSpeed = 127,

                                                .timeout = 400,
                                                .earlyExitRange = 1 });
    autonSchedule.enqueue<movepoint_c>(point{ 37, 73 },
                                       chassis,
                                       mx,
                                       my,
                                       poseCFG{ .reversed = true,
                                                .minSpeed = 30,
                                                .maxSpeed = 120,
                                                .timeout = 1000 });
    autonSchedule.enqueue<turnheading_c>(
        315, chassis, mx, my, poseCFG{ .minSpeed = 30, .timeout = 700 });
    autonSchedule.enqueue<movepoint_c>(point{ 0, 0 },
                                       chassis,
                                       mx,
                                       my,
                                       poseCFG{ .reversed = true,
                                                .minSpeed = 30,
                                                .maxSpeed = 127,
                                                .timeout = 200 });
    autonSchedule.enqueue<mech_state_c>(OUT_MIDDLE, outtake);
    autonSchedule.enqueue<movepoint_c>(point{ -999, -999 },
                                       chassis,
                                       mx,
                                       my,
                                       poseCFG{ .reversed = true,
                                                .minSpeed = 30,
                                                .maxSpeed = 40,
                                                .timeout = 1000 });
    autonSchedule.enqueue<mech_state_c>(OUT_HOARD, outtake);

    autonSchedule.enqueue<movepoint_c>(point{ 115, 115 },
                                       chassis,
                                       mx,
                                       my,
                                       poseCFG{ .reversed = false,
                                                .minSpeed = 10,
                                                .maxSpeed = 127,
                                                .timeout = 1000 });
    autonSchedule.enqueue<swing_heading_c>(
        270,
        lemlib::DriveSide::LEFT,
        chassis,
        mx,
        my,
        swingCFG{ .minSpeed = 20,
                  .dirPath = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE

        });
    // MATCHLOAD 1
    autonSchedule.enqueue<movepoint_c>(point{ 180, 125 },
                                       chassis,
                                       mx,
                                       my,
                                       poseCFG{ .reversed = false,
                                                .minSpeed = 60,
                                                .maxSpeed = 127,
                                                .timeout = 500 });
    autonSchedule.enqueue<movepoint_c>(
        point{ 200, 125 },
        chassis,
        mx,
        my,
        poseCFG{
            .reversed = false, .minSpeed = 0, .maxSpeed = 0, .timeout = 250 });
    autonSchedule.enqueue<togglePneu_c>(scraper);

    autonSchedule.enqueue<movepoint_c>(point{ 90, 125.5 },
                                       chassis,
                                       mx,
                                       my,
                                       poseCFG{ .reversed = true,
                                                .minSpeed = 10,
                                                .maxSpeed = 127,
                                                .timeout = 1000 });
    autonSchedule.enqueue<mech_state_c>(OUT_TOP, outtake);
    autonSchedule.enqueue<movepoint_c>(point{ 0, 125.5 },
                                       chassis,
                                       mx,
                                       my,
                                       poseCFG{ .reversed = true,
                                                .minSpeed = 90,
                                                .maxSpeed = 100,
                                                .timeout = 1400 });
    autonSchedule.enqueue<setPose_c>(
        lemlib::Pose{ mx * float(70_cm), my * float(122_cm), 9999 });
    autonSchedule.enqueue<mech_state_c>(OUT_HOARD, outtake);
    // autonSchedule.enqueue<movepoint_c>(point{ 999, 122 },
    //                                    chassis,
    //                                    mx,
    //                                    my,
    //                                    poseCFG{ .reversed = true,
    //                                             .minSpeed = 100,
    //                                             .maxSpeed = 127,
    //                                             .timeout = 300 });
    // ALIGN TO FIELD CROSS
    autonSchedule.enqueue<swing_heading_c>(
        179.5,
        lemlib::DriveSide::LEFT,
        chassis,
        mx,
        my,
        swingCFG{ .minSpeed = 100,
                  .dirPath = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE

        });
    autonSchedule.enqueue<movepoint_c>(point{ 59, -40 },
                                       chassis,
                                       mx,
                                       my,
                                       poseCFG{ .reversed = false,
                                                .minSpeed = 100,
                                                .maxSpeed = 127,
                                                .timeout = 1500 });

    autonSchedule.enqueue<togglePneu_c>(scraper);
    autonSchedule.enqueue<swing_heading_c>(
        220,
        lemlib::DriveSide::RIGHT,
        chassis,
        mx,
        my,
        swingCFG{ .minSpeed = 120,
                  .timeout = 1000,
                  .dirPath = lemlib::AngularDirection::CW_CLOCKWISE

        });
    autonSchedule.enqueue<movepoint_c>(point{ 120, -113 },
                                       chassis,
                                       mx,
                                       my,
                                       poseCFG{ .reversed = false,
                                                .minSpeed = 100,
                                                .maxSpeed = 127,
                                                .timeout = 1500 });
    autonSchedule.enqueue<swing_heading_c>(
        270,
        lemlib::DriveSide::RIGHT,
        chassis,
        mx,
        my,
        swingCFG{ .minSpeed = 120,
                  .timeout = 1000,
                  .earlyExitRange = 5,
                  .dirPath = lemlib::AngularDirection::CW_CLOCKWISE

        });
    autonSchedule.enqueue<movepoint_c>(point{ 180, -120 },
                                       chassis,
                                       mx,
                                       my,
                                       poseCFG{ .reversed = false,
                                                .minSpeed = 100,
                                                .maxSpeed = 127,
                                                .timeout = 750 });
    autonSchedule.enqueue<movepose_c>(lemlib::Pose{ 0, -123.6, 270 },
                                      chassis,
                                      mx,
                                      my,
                                      poseCFG{ .reversed = true,
                                               .minSpeed = 100,
                                               .maxSpeed = 127,
                                               .timeout = 750 });
    autonSchedule.enqueue<mech_state_c>(OUT_TOP, outtake);
    autonSchedule.enqueue<movepose_c>(lemlib::Pose{ 0, -123.6, 270 },
                                      chassis,
                                      mx,
                                      my,
                                      poseCFG{ .reversed = true,
                                               .minSpeed = 60,
                                               .maxSpeed = 80,
                                               .timeout = 1000 });

    // autonSchedule.enqueue<movepoint_c>(point{ 140, 98 },
    //                                    chassis,
    //                                    mx,
    //                                    my,
    //                                    poseCFG{ .reversed = true,
    //                                             .minSpeed = 10,
    //                                             .maxSpeed = 127,
    //                                             .timeout = 250 });
    // autonSchedule.enqueue<movepoint_c>(point{ 55, -95 },
    //                                    chassis,
    //                                    mx,
    //                                    my,
    //                                    poseCFG{ .reversed = true,
    //                                             .minSpeed = 10,
    //                                             .maxSpeed = 127,
    //                                             .timeout = 1000 });

    // autonSchedule.enqueue<swing_heading_c>(
    //     270,
    //     lemlib::DriveSide::RIGHT,
    //     chassis,
    //     mx,
    //     my,
    //     swingCFG{ .minSpeed = 120,
    //               .timeout = 750,
    //               .earlyExitRange = 5,
    //               .dirPath = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE

    //     });
    // autonSchedule.enqueue<movepoint_c>(point{ 0, -128 },
    //                                    chassis,
    //                                    mx,
    //                                    my,
    //                                    poseCFG{ .reversed = true,
    //                                             .minSpeed = 80,
    //                                             .maxSpeed = 127,
    //                                             .timeout = 400 });
    // autonSchedule.enqueue<mech_state_c>(OUT_TOP, outtake);
    // autonSchedule.enqueue<movepoint_c>(point{ 20, -125 },
    //                                    chassis,
    //                                    mx,
    //                                    my,
    //                                    poseCFG{ .reversed = true,
    //                                             .minSpeed = 10,
    //                                             .maxSpeed = 60,
    //                                             .timeout = 1000 });
    // autonSchedule.enqueue<setPose_c>(
    //     lemlib::Pose{ mx * float(70_cm), my * float(-122_cm), 9999 });
    // ENDCOMMENTHEREENDCOMMENTHERE |||||||||||||||||||\\
    // autonSchedule.enqueue<mech_state_c>(OUT_HOARD, outtake);
    // MATCHLOAD2
    // autonSchedule.enqueue<movepose_c>(lemlib::Pose{ 160, -123, 270 },
    //                                   chassis,
    //                                   mx,
    //                                   my,
    //                                   poseCFG{ .reversed = false,
    //                                            .minSpeed = 100,
    //                                            .maxSpeed = 127,
    //                                            .timeout = 500 });
    // autonSchedule.enqueue<movepose_c>(lemlib::Pose{ 200, -123, 270 },
    //                                   chassis,
    //                                   mx,
    //                                   my,
    //                                   poseCFG{ .reversed = false,
    //                                            .minSpeed = 80,
    //                                            .maxSpeed = 90,
    //                                            .timeout = 500 });
    // autonSchedule.enqueue<movepose_c>(lemlib::Pose{ 75, -124.5, 270 },
    //                                   chassis,
    //                                   mx,
    //                                   my,
    //                                   poseCFG{ .reversed = true,
    //                                            .minSpeed = 120,
    //                                            .maxSpeed = 127,
    //                                            .timeout = 800 });
    // autonSchedule.enqueue<mech_state_c>(OUT_TOP, outtake);

    while (autonSchedule.tick()) {
        outtake.loop();
        pros::delay(10);
        // std::cout << chassis.getPose().x << std::endl;
    }
    std::cout << double(pros::millis() - iteration) / 1000 << std::endl;
}

void quarterAuto(bool isBlue) {
    float mx = isBlue ? 1 : -1;
    float my = isBlue ? -1 : 1;
    int iteration = pros::millis();
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    autonSchedule.enqueue<setPose_c>(
        lemlib::Pose{ mx * float(118.11_cm),
                      my * float(18.415_cm),
                      adjustHeading(90, mx, my) }); // y:38
    autonSchedule.enqueue<togglePneu_c>(descorer_l);

    autonSchedule.enqueue<mech_state_c>(OUT_HOARD, outtake);
    autonSchedule.enqueue<swing_heading_c>(
        55,
        lemlib::DriveSide::LEFT,
        chassis,
        mx,
        my,
        swingCFG{ .minSpeed = 40,
                  .earlyExitRange = 3,
                  .dirPath = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE

        });

    autonSchedule.enqueue<movepoint_c>(point{ 35, 96 },
                                       chassis,
                                       mx,
                                       my,
                                       poseCFG{ .minSpeed = 30,
                                                .maxSpeed = 127,

                                                .timeout = 2000,
                                                .earlyExitRange = 7 });
    autonSchedule.enqueue<togglePneu_c>(scraper);
    autonSchedule.enqueue<movepoint_c>(point{ 30, 100 },
                                       chassis,
                                       mx,
                                       my,
                                       poseCFG{ .minSpeed = 30,
                                                .maxSpeed = 127,

                                                .timeout = 420,
                                                .earlyExitRange = 1 });
    autonSchedule.enqueue<movepoint_c>(point{ 37, 73 },
                                       chassis,
                                       mx,
                                       my,
                                       poseCFG{ .reversed = true,
                                                .minSpeed = 30,
                                                .maxSpeed = 120,
                                                .timeout = 1000 });
    autonSchedule.enqueue<turnheading_c>(
        315, chassis, mx, my, poseCFG{ .minSpeed = 30, .timeout = 700 });
    autonSchedule.enqueue<movepoint_c>(point{ 0, 0 },
                                       chassis,
                                       mx,
                                       my,
                                       poseCFG{ .reversed = true,
                                                .minSpeed = 30,
                                                .maxSpeed = 127,
                                                .timeout = 200 });
    autonSchedule.enqueue<mech_state_c>(OUT_MIDDLE, outtake);
    autonSchedule.enqueue<movepoint_c>(
        point{ -999, -999 },
        chassis,
        mx,
        my,
        poseCFG{
            .reversed = true, .minSpeed = 30, .maxSpeed = 40, .timeout = 750 });
    autonSchedule.enqueue<mech_state_c>(OUT_HOARD, outtake);

    autonSchedule.enqueue<movepoint_c>(point{ 115, 115 },
                                       chassis,
                                       mx,
                                       my,
                                       poseCFG{ .reversed = false,
                                                .minSpeed = 10,
                                                .maxSpeed = 127,
                                                .timeout = 1000 });
    autonSchedule.enqueue<swing_heading_c>(
        270,
        lemlib::DriveSide::LEFT,
        chassis,
        mx,
        my,
        swingCFG{ .minSpeed = 20,
                  .dirPath = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE

        });
    // MATCHLOAD 1
    autonSchedule.enqueue<movepoint_c>(point{ 180, 125 },
                                       chassis,
                                       mx,
                                       my,
                                       poseCFG{ .reversed = false,
                                                .minSpeed = 60,
                                                .maxSpeed = 127,
                                                .timeout = 500 });
    autonSchedule.enqueue<movepoint_c>(
        point{ 200, 125 },
        chassis,
        mx,
        my,
        poseCFG{
            .reversed = false, .minSpeed = 0, .maxSpeed = 0, .timeout = 500 });
    autonSchedule.enqueue<togglePneu_c>(scraper);

    autonSchedule.enqueue<movepoint_c>(point{ 90, 125.5 },
                                       chassis,
                                       mx,
                                       my,
                                       poseCFG{ .reversed = true,
                                                .minSpeed = 10,
                                                .maxSpeed = 127,
                                                .timeout = 1000 });
    autonSchedule.enqueue<mech_state_c>(OUT_TOP, outtake);
    autonSchedule.enqueue<movepoint_c>(point{ 0, 125.5 },
                                       chassis,
                                       mx,
                                       my,
                                       poseCFG{ .reversed = true,
                                                .minSpeed = 90,
                                                .maxSpeed = 100,
                                                .timeout = 2000 });
    autonSchedule.enqueue<setPose_c>(
        lemlib::Pose{ mx * float(70_cm), my * float(122_cm), 9999 });
    autonSchedule.enqueue<movepoint_c>(point{ 120, 90 },
                                       chassis,
                                       mx,
                                       my,
                                       poseCFG{ .reversed = false,
                                                .minSpeed = 0,
                                                .maxSpeed = 127,
                                                .timeout = 2000 });
    autonSchedule.enqueue<turnheading_c>(
        270, chassis, mx, my, poseCFG{ .minSpeed = 30, .timeout = 700 });
    autonSchedule.enqueue<togglePneu_c>(descorer_l);

    autonSchedule.enqueue<movepoint_c>(
        point{ 45, 90 },
        chassis,
        mx,
        my,
        poseCFG{
            .reversed = true, .minSpeed = 0, .maxSpeed = 90, .timeout = 2000 });
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
    std::cout << double(pros::millis() - iteration) / 1000 << std::endl;
}