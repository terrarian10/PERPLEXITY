#include "main.h"

#include "pros/abstract_motor.hpp"
#include "pros/rtos.hpp"
#include "robodash/api.h" // IWYU pragma: export
#include <cstddef>

// Chassis constructor
pros::MotorGroup left_motors({ 11, -1, -2 }, pros::MotorGearset::blue);
pros::MotorGroup right_motors({ -20, 19, 10 }, pros::MotorGearset::blue);
lemlib::Drivetrain drivetrain(&left_motors,  // left motor group
                              &right_motors, // right motor group
                              10.75,         // 10 inch track width
                              lemlib::Omniwheel::NEW_325,
                              360, // drivetrain rpm is 360
                              2    // horizontal drift is 2 (for now)
);

pros::Motor outt_1(5, pros::MotorGearset::green);
pros::Motor outt_2(6, pros::MotorGearset::green);
pros::Motor outt_3(7, pros::MotorGearset::green);

Outtake outtake(outt_1, outt_2, outt_3, Outtake::State::OFF, 12000);

lemlib::OdomSensors sensors(
    nullptr, // vertical tracking wheel 1, set to nullptr
    nullptr, // vertical tracking wheel 2
    nullptr, // horizontal tracking wheel 1
    nullptr, // horizontal tracking wheel 2
    nullptr  // imu
);
// lateral PID controller
lemlib::ControllerSettings lateral_controller(
    20,  // proportional gain (kP)
    0,   // integral gain (kI)
    80,  // derivative gain (kD)
    3,   // anti windup
    1,   // small error range, in inches
    100, // small error range timeout, in milliseconds
    3,   // large error range, in inches
    500, // large error range timeout, in milliseconds
    20   // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(
    4,   // proportional gain (kP)
    0,   // integral gain (kI)
    29,  // derivative gain (kD)
    3,   // anti windup
    1,   // small error range, in degrees
    100, // small error range timeout, in milliseconds
    3,   // large error range, in degrees
    500, // large error range timeout, in milliseconds
    0    // maximum acceleration (slew)
);
// create the chassis
lemlib::Chassis chassis(drivetrain,         // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors             // odometry sensors
);

// Create Controller
pros::Controller master(pros::E_CONTROLLER_MASTER);

/**                                                                            \
 * Runs initialization code. This occurs as soon as the program is started.    \
 *                                                                             \
 * All other competition modes are blocked by initialize; it is recommended    \
 * to keep execution time for this mode under a few seconds.                   \
 */
rd::Selector selector({
    { "auton_skills", auton_skills },
    { "autons_positive_red", autons_positive_red },
    { "autons_positive_blue", autons_positive_blue },
    { "autons_negative_red", autons_negative_red },
    { "autons_negative_blue", autons_negative_blue },
});

void initialize() {
    // imu.reset();
    chassis.calibrate(true);
    // outtake.move(Outtake::State::OFF);

    // Initialize chassis
    master.rumble(".");
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
    // . . .
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
    // . . .
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
    // Set the break mode for the autonomous
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);

    // run auton selector
    selector.run_auton();

    // Run specific auton, used for testing
    //  auton_skills();
    // autons_positive_red();
    // autons_positive_blue();
    //  autons_negative_red();
    // autons_negative_blue();
}

lemlib::ExpoDriveCurve throttle(3, 12, 1.05);
lemlib::ExpoDriveCurve steerCurve(3, 12, 1.05);

namespace pros {
#define E_CONTROLLER_DIGITAL_L0 E_CONTROLLER_DIGITAL_RIGHT
#define E_CONTROLLER_DIGITAL_R0 E_CONTROLLER_DIGITAL_Y
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */

void opcontrol() {
    // FIXME: REMOVE - bodge
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_A) &&
        master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)) {
        autonomous();
        return;
    } // Forcibly runs the autonomous, for debugging

    // This is preference to what you like to drive on
    pros::motor_brake_mode_e_t driver_preference_brake = MOTOR_BRAKE_COAST;

    chassis.setBrakeMode(driver_preference_brake);
    // opcontrol loop
    while (true) {
        // std::cout << "Running" << std::endl;
        //  Get how far the joysticks are moved
        int leftY = (master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));
        int rightY = (master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y));
        // turn it into tank drive
        chassis.tank(leftY, rightY);
        // delay a small amount to prevent brain overload and improve timer
        // accuracy
        pros::delay(10); // This is used for timer calculations!
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
            outtake.move(Outtake::State::HOARD);
        } else if (master.get_digital_new_press(
                       pros::E_CONTROLLER_DIGITAL_L2)) {
            outtake.move(Outtake::State::OFF);
        } else if (master.get_digital_new_press(
                       pros::E_CONTROLLER_DIGITAL_R1)) {
            outtake.move(Outtake::State::TOP);
        } else if (master.get_digital_new_press(
                       pros::E_CONTROLLER_DIGITAL_R2)) {
            outtake.move(Outtake::State::MIDDLE);
        }
    }
}