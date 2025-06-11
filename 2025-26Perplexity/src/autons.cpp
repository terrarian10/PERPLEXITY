#include "autons.hpp"
//#include "Intake.hpp"
#include "main.h"
#include <cmath>
#include <cstdint>
#include <iostream>
// /////
// // For installation, upgrading, documentations, and tutorials, check out our
// // website! https://ez-robotics.github.io/EZ-Template/
// /////

// These are out of 127
const int DRIVE_SPEED = 110;
const int TURN_SPEED = 90;
const int SWING_SPEED = 90;

void absTurn(double angle, bool blocking = true, int timeout = 750) {
    // std::cout << "theta:" << operator""_deg(angle)
    // << " new x:" << 1e5 * std::sin(operator""_deg(angle))
    // << " new y:" << 1e5 * std::cos(operator""_deg(angle)) << '\n';
    chassis.turnToHeading(angle, timeout);

    if (blocking) chassis.waitUntilDone();
}

void relativeMove(double distance, bool blocking = true, int timeout = 4000) {
    std::cerr << "in function\v";
    lemlib::Pose pose = chassis.getPose(true);

    std::cerr << "moving from x:" << pose.x << " y:" << pose.y
              << " theta:" << pose.theta << '\v'
              << "Moving to x:" << pose.x + distance * std::sin(pose.theta)
              << " y:" << pose.y + distance * std::cos(pose.theta) << '\n';
    // std::cout << "x:" << pose.x << " y:" << pose.y << " theta:" << pose.theta
    // << " dist: " << distance
    // << " new x:" << pose.x + distance * std::sin(pose.theta)
    // << " new y:" << pose.y + distance * std::cos(pose.theta) << '\n';
    chassis.moveToPoint(pose.x + distance * std::sin(pose.theta),
                        pose.y + distance * std::cos(pose.theta),
                        timeout,
                        { distance > 0 });
    if (blocking) chassis.waitUntilDone();
}

// ///
// // Constants
// ///
// void default_constants() {
//     // chassis.pid_heading_constants_set(11, 0, 20);
//     // chassis.pid_drive_constants_set(20, 0, 100);
//     // chassis.pid_turn_constants_set(3, 0.05, 20, 15);
//     // chassis.pid_swing_constants_set(6, 0, 65);
//     // chassis.pid_turn_exit_condition_set(
//     //     80_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
//     // chassis.pid_swing_exit_condition_set(
//     //     80_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
//     // chassis.pid_drive_exit_condition_set(
//     //     80_ms, 1_in, 250_ms, 3_in, 500_ms, 500_ms);
//     // chassis.pid_turn_chain_constant_set(3_deg);
//     // chassis.pid_swing_chain_constant_set(5_deg);
//     // chassis.pid_drive_chain_constant_set(3_in);
//     // chassis.slew_drive_constants_set(7_in, 80);
// }
void autons_touch() {
    uint32_t start_time = pros::millis();
    pros::delay(10_sec);
    chassis.setPose(60, 23.598, 90);
    std::cerr << "here\n";
    chassis.moveToPoint(42, 23.601, 6000, { false, 60 });
    chassis.waitUntil(3_ft);

    std::cout << "\033[7mTIME time: " << (pros::millis() - start_time) / 1000.0
              << " sec\033[0m\n";
}

void auton_skills() {
    uint32_t start_time = pros::millis();

    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    // chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(0, 20, 100000);
    // chassis.moveToPoint(-47, -24, 1);
    std::cout << "\033[7mTIME time: " << (pros::millis() - start_time) / 1000.0
              << " sec\033[0m\n";
}

void autons_positive_red() {
    
}

void autons_positive_blue() {
    
}

void autons_negative_red() {
    
}

void autons_negative_blue() {
    
}

/* ///
// Drive Example
///
void drive_example() {
    // The first parameter is target inches
    // The second parameter is max speed the robot will drive at
    // The third parameter is a boolean (true or false) for
    enabling/disabling a
    // slew at the start of drive motions for slew, only enable it when the
    // // drive distance is greater than the slew distance + a few inches
    // chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
    // chassis.pid_wait();
    // chassis.pid_drive_set(-12_in, DRIVE_SPEED);
    // chassis.pid_wait();
    // chassis.pid_drive_set(-12_in, DRIVE_SPEED);
    // chassis.pid_wait();
}
///
// Turn Example
///
void turn_example() {
    // The first parameter is the target in degrees
    // The second parameter is max speed the robot will drive at
    // chassis.pid_turn_set(90_deg, TURN_SPEED);
    // chassis.pid_wait();
    // chassis.pid_turn_set(45_deg, TURN_SPEED);
    // chassis.pid_wait();
    // chassis.pid_turn_set(0_deg, TURN_SPEED);
    // chassis.pid_wait();
}
///
// Combining Turn + Drive
///
void drive_and_turn() {
    chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
    chassis.pid_wait();
    chassis.pid_turn_set(45_deg, TURN_SPEED);
    chassis.pid_wait();
    chassis.pid_turn_set(-45_deg, TURN_SPEED);
    chassis.pid_wait();
    chassis.pid_turn_set(0_deg, TURN_SPEED);
    chassis.pid_wait();
    chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
    chassis.pid_wait();
}
void skills() {
    // drive towards mobile goal
    chassis.pid_drive_set(-18_in, DRIVE_SPEED);
    chassis.pid_wait_quick();
    backClamp.extend();
    pros::delay(250);
    // turn intake on
    intake.move(Intake::State::ANY);
    chassis.pid_drive_set(3_in, DRIVE_SPEED);
    chassis.pid_wait();
    // clamp onto mobile goal
    // turn so intake faces direction of rings to drive towards
    pros::delay(250);
    chassis.pid_turn_set(-90_deg, TURN_SPEED);
    chassis.pid_wait();
    // drive towards rings on right side of starting point of robot
    chassis.pid_drive_set(36_in, 0.5 * DRIVE_SPEED);
    chassis.pid_wait();
    pros::delay(1000);
    chassis.pid_turn_set(-195_deg, TURN_SPEED);
    chassis.pid_wait();
    intake.move(Intake::State::STOP);
    chassis.pid_drive_set(-18_in, DRIVE_SPEED);
    chassis.pid_wait();
    backClamp.retract();
    chassis.pid_drive_set(12_in, DRIVE_SPEED);
    chassis.pid_wait();
    return;
    chassis.pid_turn_set(-30_deg, TURN_SPEED);
    chassis.pid_wait();
    chassis.pid_drive_set(29_in, DRIVE_SPEED);
    chassis.pid_wait();
    chassis.pid_turn_set(90_deg, TURN_SPEED);
    chassis.pid_wait();
    chassis.pid_drive_set(6_in, DRIVE_SPEED);
    chassis.pid_wait();
    chassis.pid_turn_set(-20_deg, TURN_SPEED);
    chassis.pid_wait();
    chassis.pid_drive_set(10_in, DRIVE_SPEED);
    chassis.pid_wait();
    chassis.pid_turn_set(70_deg, TURN_SPEED);
    chassis.pid_wait();
    chassis.pid_drive_set(-20_in, DRIVE_SPEED);
    chassis.pid_wait();
    // release backclamp in positive corner
    backClamp.retract();
    // drive towards left side of starting point of robot
    chassis.pid_drive_set(50_in, DRIVE_SPEED);
    chassis.pid_wait();
    chassis.pid_turn_set(-20_deg, TURN_SPEED);
    chassis.pid_wait();
    chassis.pid_turn_set(180_deg, TURN_SPEED);
    chassis.pid_wait();
    chassis.pid_drive_set(-48_in, DRIVE_SPEED);
    chassis.pid_wait();
    // clamp mobile goal
    backClamp.extend();
    // turn to face rings on this side
    chassis.pid_turn_set(-90_deg, TURN_SPEED);
    chassis.pid_wait();
    chassis.pid_drive_set(30_in, DRIVE_SPEED);
    chassis.pid_wait();
    chassis.pid_turn_set(-85_deg, TURN_SPEED);
    chassis.pid_wait();
    chassis.pid_drive_set(30_in, DRIVE_SPEED);
    chassis.pid_wait();
    chassis.pid_turn_set(-70_deg, TURN_SPEED);
    chassis.pid_wait();
    chassis.pid_drive_set(22_in, DRIVE_SPEED);
    chassis.pid_wait();
    chassis.pid_turn_set(-30_deg, TURN_SPEED);
    chassis.pid_wait();
    chassis.pid_drive_set(6_in, DRIVE_SPEED);
    chassis.pid_wait();
    chassis.pid_turn_set(90_deg, TURN_SPEED);
    chassis.pid_wait();
    chassis.pid_drive_set(10_in, DRIVE_SPEED);
    chassis.pid_wait();
    chassis.pid_turn_set(-90_deg, TURN_SPEED);
    chassis.pid_wait();
    chassis.pid_drive_set(100_in, DRIVE_SPEED);
    chassis.pid_wait();
    chassis.pid_turn_set(-180_deg, TURN_SPEED);
    chassis.pid_wait();
    // score in positive corner
    chassis.pid_drive_set(-20_in, DRIVE_SPEED);
    chassis.pid_wait();
    backClamp.retract();
    // stop intake
    intake.move(Intake::State::STOP);
}
void matchAutonRed() {
    // robot needs to start at an angle
    chassis.pid_drive_set(-2.65_ft, DRIVE_SPEED * .75);
    chassis.pid_wait();
    backClamp.extend();
    // clamp mobile goal
    // turn on intake
    intake.move(Intake::State::RED);
    pros::delay(1000);
    chassis.pid_turn_set(90_deg, TURN_SPEED);
    chassis.pid_wait();
    chassis.pid_drive_set(1.25_ft, DRIVE_SPEED);
    chassis.pid_wait();
    return;
    chassis.pid_turn_set(45_deg, TURN_SPEED);
    chassis.pid_wait();
    chassis.pid_drive_set(24_in, DRIVE_SPEED);
    chassis.pid_wait();
    chassis.pid_turn_set(90_deg, TURN_SPEED);
    chassis.pid_wait();
    chassis.pid_drive_set(24_in, DRIVE_SPEED);
    chassis.pid_wait();
    chassis.pid_drive_set(-10_in, DRIVE_SPEED);
    chassis.pid_wait();
    chassis.pid_turn_set(45_deg, TURN_SPEED);
    chassis.pid_wait();
    chassis.pid_drive_set(15_in, DRIVE_SPEED);
    chassis.pid_wait();
    chassis.pid_turn_set(45_deg, TURN_SPEED);
    chassis.pid_wait();
    chassis.pid_drive_set(30_in, DRIVE_SPEED);
    chassis.pid_wait();
    // retract
    // backClamp.retract();
    //  turn off intake
    intake.move(Intake::State::STOP);
}
void matchAutonBlue() {
    // robot needs to start at an angle
    chassis.pid_drive_set(-2.6_ft, DRIVE_SPEED * .75);
    chassis.pid_wait();
    backClamp.extend();
    // clamp mobile goal
    // turn on intake
    intake.move(Intake::State::BLUE);
    pros::delay(1000);
    chassis.pid_drive_set(2_ft, DRIVE_SPEED);
    chassis.pid_wait();
    return;
    chassis.pid_drive_set(-39_in, DRIVE_SPEED);
    chassis.pid_wait();
    backClamp.extend();
    // turn on intake
    intake.move(Intake::State::BLUE);
    chassis.pid_turn_set(-45_deg, TURN_SPEED);
    chassis.pid_wait();
    chassis.pid_drive_set(24_in, DRIVE_SPEED);
    chassis.pid_wait();
    chassis.pid_turn_set(-90_deg, TURN_SPEED);
    chassis.pid_wait();
    chassis.pid_drive_set(24_in, DRIVE_SPEED);
    chassis.pid_wait();
    chassis.pid_drive_set(-10_in, DRIVE_SPEED);
    chassis.pid_wait();
    chassis.pid_turn_set(-45_deg, TURN_SPEED);
    chassis.pid_wait();
    chassis.pid_drive_set(15_in, DRIVE_SPEED);
    chassis.pid_wait();
    chassis.pid_turn_set(-45_deg, TURN_SPEED);
    chassis.pid_wait();
    chassis.pid_drive_set(30_in, DRIVE_SPEED);
    chassis.pid_wait();
    // turn off intake
    intake.move(Intake::State::STOP);
}
///
// Wait Until and Changing Max Speed
///
void wait_until_change_speed() {
    // pid_wait_until will wait until the robot gets to a desired position
    // When the robot gets to 6 inches slowly, the robot will travel the
    // remaining distance at full speed
    chassis.pid_drive_set(24_in, 30, true);
    chassis.pid_wait_until(6_in);
    chassis.pid_speed_max_set(
        DRIVE_SPEED); // After driving 6 inches at 30 speed, the robot will
        go
                      // the remaining distance at DRIVE_SPEED
    chassis.pid_wait();
    chassis.pid_turn_set(45_deg, TURN_SPEED);
    chassis.pid_wait();
    chassis.pid_turn_set(-45_deg, TURN_SPEED);
    chassis.pid_wait();
    chassis.pid_turn_set(0_deg, TURN_SPEED);
    chassis.pid_wait();
    // When the robot gets to -6 inches slowly, the robot will travel the
    // remaining distance at full speed
    chassis.pid_drive_set(-24_in, 30, true);
    chassis.pid_wait_until(-6_in);
    chassis.pid_speed_max_set(
        DRIVE_SPEED); // After driving 6 inches at 30 speed, the robot will
        go
                      // the remaining distance at DRIVE_SPEED
    chassis.pid_wait();
}

///
// Swing Example
///
void swing_example() {
    // The first parameter is ez::LEFT_SWING or ez::RIGHT_SWING
    // The second parameter is the target in degrees
    // The third parameter is the speed of the moving side of the drive
    // The fourth parameter is the speed of the still side of the drive, this
    // allows for wider arcs
    chassis.pid_swing_set(ez::LEFT_SWING, 45_deg, SWING_SPEED, 45);
    chassis.pid_wait();
    chassis.pid_swing_set(ez::RIGHT_SWING, 0_deg, SWING_SPEED, 45);
    chassis.pid_wait();
    chassis.pid_swing_set(ez::RIGHT_SWING, 45_deg, SWING_SPEED, 45);
    chassis.pid_wait();
    chassis.pid_swing_set(ez::LEFT_SWING, 0_deg, SWING_SPEED, 45);
    chassis.pid_wait();
}

///
// Motion Chaining
///
void motion_chaining() {
    // Motion chaining is where motions all try to blend together instead of
    // individual movements. This works by exiting while the robot is still
    // moving a little bit. To use this, replace pid_wait with
    // pid_wait_quick_chain.
    chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
    chassis.pid_wait();
    chassis.pid_turn_set(45_deg, TURN_SPEED);
    chassis.pid_wait_quick_chain();
    chassis.pid_turn_set(-45_deg, TURN_SPEED);
    chassis.pid_wait_quick_chain();
    chassis.pid_turn_set(0_deg, TURN_SPEED);
    chassis.pid_wait();
    // Your final motion should still be a normal pid_wait
    chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
    chassis.pid_wait();
}

///
// Auto that tests everything
///
void combining_movements() {
    chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
    chassis.pid_wait();
    chassis.pid_turn_set(45_deg, TURN_SPEED);
    chassis.pid_wait();
    chassis.pid_swing_set(ez::RIGHT_SWING, -45_deg, SWING_SPEED, 45);
    chassis.pid_wait();
    chassis.pid_turn_set(0_deg, TURN_SPEED);
    chassis.pid_wait();
    chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
    chassis.pid_wait();
}

///
// Interference example
///
void tug(int attempts) {
    for (int i = 0; i < attempts - 1; i++) {
        // Attempt to drive backward
        printf("i - %i", i);
        chassis.pid_drive_set(-12_in, 127);
        chassis.pid_wait();
        // If failsafed...
        if (chassis.interfered) {
            chassis.drive_sensor_reset();
            chassis.pid_drive_set(-2_in, 20);
            pros::delay(1000);
        }
        // If the robot successfully drove back, return
        else {
            return;
        }
    }
}

// If there is no interference, the robot will drive forward and turn 90
// degrees. If interfered, the robot will drive forward and then attempt to
// drive backward.
void interfered_example() {
    chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
    chassis.pid_wait();
    if (chassis.interfered) {
        tug(3);
        return;
    }
    chassis.pid_turn_set(90_deg, TURN_SPEED);
    chassis.pid_wait();
}

// . . .
// Make your own autonomous functions here!
// . . . */
