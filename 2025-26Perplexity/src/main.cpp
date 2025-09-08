#include "main.h"
#include "Outtake.hpp"
#include "airCylinder.hpp"
#include "color_sort.hpp"
#include "consts.h"
#include "controller_data.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/optical.hpp"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"
#include "robodash/api.h" // IWYU pragma: export
#include <cstdlib>
#include <vector>

// Chassis constructor
pros::MotorGroup left_motors({ -4, -9, 3 }, pros::MotorGearset::blue);
pros::MotorGroup right_motors({ -2, 13, 11 }, pros::MotorGearset::blue);
// Piggyback off of purdues hard work
lemlib::Drivetrain drivetrain(&left_motors,  // left motor group
                              &right_motors, // right motor group
                              10.75,         // 10 inch track width
                              lemlib::Omniwheel::NEW_325,
                              360, // drivetrain rpm is 360
                              2    // horizontal drift is 2 (for now)
);
bool isRed;
pros::Optical optical(10);
ColourDetector colorDetector(optical);

// Initialize the Scraper
AirCylinder scraper('h');
// Make attacher wirj
AirCylinder attacher('g');
// Doubleparrk
AirCylinder double_park('e');
pros::IMU imu(19);
pros::Rotation horizOdom();
// Literally anything  is better than this method
// Don't touch it it works
pros::Motor outt_1(5, pros::MotorGearset::blue);
pros::Motor outt_2(6, pros::MotorGearset::green);
pros::Motor outt_3(7, pros::MotorGearset::green);
std::vector<pros::Motor> test = { outt_1, outt_2, outt_3 };
mecha_control outtake_ctrl = {
    { { { { 0, -1 }, { 1, -1 }, { 2, -1 } }, Outt_States::TOP },
      { { { 0, -1 }, { 1, 1 }, { 2, -1 } }, Outt_States::MIDDLE },
      { { { 0, 1 }, { 1, 1 }, { 2, -1 } }, Outt_States::BOTTOM },
      { { { 0, -1 }, { 1, -1 }, { 2, 1 } }, Outt_States::BOTTOM_STORE },
      { { { 0, 0 }, { 1, 0 }, { 2, 0 } }, Outt_States::OFF },
      { { { 0, -1 }, { 1, -1 }, { 2, 0 } }, Outt_States::TOP_STORE } },
    "Outtake"
};
Outtake outtake(test, colorDetector, outtake_ctrl, Outt_States::OFF, 12000);
pros::Rotation horizontalEncoder(20);

lemlib::TrackingWheel horizontal(&horizontalEncoder,
                                 lemlib::Omniwheel::NEW_275,
                                 -2);

// The sensors are imaginary
lemlib::OdomSensors sensors(
    nullptr,     // vertical tracking wheel 1, set to nullptr
    nullptr,     // vertical tracking wheel 2
    &horizontal, // horizontal tracking wheel 1
    nullptr,     // horizontal tracking wheel 2
    &imu         // imu
);

// Guess and check final boss
// lateral PID controller
lemlib::ControllerSettings lateral_controller(
    10,  // proportional gain (kP)
    0,   // integral gain (kI)
    3,   // derivative gain (kD)
    3,   // anti windup
    1,   // small error range, in inches
    100, // small error range timeout, in milliseconds
    3,   // large error range, in inches
    500, // large error range timeout, in milliseconds
    20   // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(
    2,   // proportional gain (kP)
    0,   // integral gain (kI)
    10,  // derivative gain (kD)
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

std::vector<ModularControl::macro> macros;

// Create Controller
pros::Controller master(pros::E_CONTROLLER_MASTER);

ModularControl bartholomew(chassis,
                           master,
                           outtake,
                           1); // His name is bartholomew. Please respect
                               // Bartholomew

/**                                                                            \
 * Runs initialization code. This occurs as soon as the program is started.    \
 *                                                                             \
 * All other competition modes are blocked by initialize; it is recommended    \
 * to keep execution time for this mode under a few seconds.                   \
 */

// It looks nice
rd::Selector selector({
    { "auton_skills", auton_skills },
    { "autons_positive_red", autons_positive_red },
    { "autons_positive_blue", autons_positive_blue },
    { "autons_negative_red", autons_negative_red },
    { "autons_negative_blue", autons_negative_blue },
});

void initialize_macros() {
    macros.reserve(2);
    macros.push_back({ { { -157, -118.9, 270, true },
                         { -157, 118.9, 270, true },
                         { 157, 118.9, 90, false },
                         { 157, -118.9, 90, false } },
                       "MATCHLOAD" });
    macros.push_back({ { { -157, -118.9, 270, true },
                         { -157, 118.9, 270, true },
                         { 157, 118.9, 90, false },
                         { 157, -118.9, 90, false } },
                       "TEST" });
}
// Make sure bot is ready
void initialize() {
    chassis.calibrate(true);
    outt_1.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    initialize_macros();
    // Initialize chassis and macros
    master.rumble(".");
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
// Why would we need this
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
// Useless
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
    // watch code implode
    selector.run_auton();

    // Run specific auton, used for testing
    //  auton_skills();
    // autons_positive_red();
    // autons_positive_blue();
    //  autons_negative_red();
    // autons_negative_blue();
}
// Curve stuff good
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
    bartholomew.updateDisplay(macros[bartholomew.get_active_address() - 1]);
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_A) &&
        master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)) {
        autonomous();
        return;
    } // Forcibly runs the autonomous, for debugging

    // This is preference to what you like to drive on, because ofc it can be 2
    // seperate lines
    pros::motor_brake_mode_e_t driver_preference_brake =
        pros::E_MOTOR_BRAKE_COAST;
    bool canUpdateMacros;
    chassis.setBrakeMode(driver_preference_brake);
    int iteration = 0;
    // watch afshin implode the bot
    while (true) {
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_A) &&
            master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)) {
            autonomous();
            return;
        } // Forcibly runs the autonomous, for debugging

        if (colorDetector.get_color() == colorDetector.RED &&
            outtake.get_state() != Outt_States::OFF) {
            std::cout << colorDetector.get_proximity() << std::endl;
            outtake.emergency(350, { { 1, -1 }, { 1, 1 }, { 1, 1 } });
        }
        if (iteration % 10 == 0) { master.clear(); }
        if (iteration % 5 == 0) {
            bartholomew.updateDisplay(
                macros[bartholomew.get_active_address() - 1]);
        }
        //  Get how far the joysticks are moved
        int leftY = (master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));
        int rightY = (master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y));
        // make it TENK
        chassis.tank(leftY, rightY);
        // delay a small amount to prevent brain overload and improve timer
        // accuracy
        pros::delay(10); // Timer calculations and not making the brain into the
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
            bartholomew.activateMacro(
                macros[bartholomew.get_active_address() - 1], isRed);
        }
        // 9/10 programmers quit before finding a stupider method to achieve
        // some non needed funciton
        if (std::abs(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X)) >
            115) {
            if (canUpdateMacros) {
                bartholomew.incrementAddress(
                    std::abs(
                        master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X)) /
                    master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X));
                if (bartholomew.get_active_address() < 1) {
                    bartholomew.incrementAddress(
                        macros.size() +
                        (bartholomew.get_active_address() * -1));
                } else if (bartholomew.get_active_address() > macros.size()) {
                    bartholomew.incrementAddress(
                        (bartholomew.get_active_address() * -1) + 1);
                }
                bartholomew.updateDisplay(
                    macros[bartholomew.get_active_address() - 1]);
                canUpdateMacros = false;
            }
        } else {
            canUpdateMacros = true;
        }
        // first portable fusion reactor
        // Outtake is actual insanity please fix. It works I guess
        // I dare someone to find a more inefficient way to do this
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
            if (outtake.get_state() == Outt_States::BOTTOM_STORE) {
                outtake.run_at_state(Outt_States::TOP_STORE);
            } else if (outtake.get_state() == Outt_States::TOP_STORE) {
                outtake.run_at_state(Outt_States::OFF);

            } else {
                outtake.run_at_state(Outt_States::BOTTOM_STORE);
            }

        } else if (master.get_digital_new_press(
                       pros::E_CONTROLLER_DIGITAL_L2)) {
            if (outtake.get_state() == Outt_States::BOTTOM) {
                outtake.run_at_state(Outt_States::OFF);
            } else {
                outtake.run_at_state(Outt_States::BOTTOM);
            }
        } else if (master.get_digital_new_press(
                       pros::E_CONTROLLER_DIGITAL_R1)) {
            if (outtake.get_state() == Outt_States::TOP) {
                outtake.run_at_state(Outt_States::OFF);
            } else {
                outtake.run_at_state(Outt_States::TOP);
            }
        } else if (master.get_digital_new_press(
                       pros::E_CONTROLLER_DIGITAL_R2)) {
            if (outtake.get_state() == Outt_States::MIDDLE) {
                outtake.run_at_state(Outt_States::OFF);
            } else {
                outtake.run_at_state(Outt_States::MIDDLE);
            }
        }
        // Toggle funny scrapers
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
            scraper.toggle();
        }

        // "Double-Park-Thingy" as I was told
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
            double_park.toggle();
        }
        // std::cout << colorDetector.get_proximity() << '\n';
        iteration++;
    }
}