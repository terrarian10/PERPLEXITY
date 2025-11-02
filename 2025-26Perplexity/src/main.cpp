#include "main.h"
#include "Outtake.hpp"
#include "airCylinder.hpp"
#include "autons.hpp"
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
#include <cstddef>
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
AirCylinder scraper('h', false);
// Make attacher wirj
// AirCylinder attacher('g');
// Doubleparrk
AirCylinder descorer('a');
// pros::IMU imu(19);
pros::Rotation horizOdom(16);
// Literally anything  is better than this method
// Don't touch it it works
pros::Motor outt_1(9, pros::MotorGearset::blue);
pros::Motor outt_2(6, pros::MotorGearset::blue);
std::vector<pros::Motor> test = { outt_1, outt_2 };
// AirCylinder scoring_cylinder('h');
std::vector<AirCylinder> intake_cylinders = {};
mecha_control outtake_ctrl = {
    { { { { 0, 1 }, { 1, 1 } }, Outt_States::TOP },
      { { { 0, -1 }, { 1, 0.25 } }, Outt_States::MIDDLE },
      { { { 0, 1 }, { 1, 1 } }, Outt_States::BOTTOM },
      { { { 0, 0 }, { 1, 0 }, { 2, 0 } }, Outt_States::OFF } },
    "Outtake"
};
Outtake outtake(test,
                intake_cylinders,
                colorDetector,
                outtake_ctrl,
                Outt_States::OFF,
                6000);
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
    nullptr      //&imu         // imu
);

// Guess and check final boss
// lateral PID controller
lemlib::ControllerSettings lateral_controller(
    75,  // proportional gain (kP) - 55 60 65 70 75 80
    0,   // integral gain (kI)
    34,  // derivative gain (kD) -- 24 32 32 33 34 36
    3,   // anti windup3
    0.5, // small error range, in inches1
    100, // small error range timeout, in milliseconds100
    1,   // large error range, in inches3
    500, // large error range timeout, in milliseconds500
    20   // maximum acceleration (slew)20
);

// angular PID controller
lemlib::ControllerSettings angular_controller(
    6,   // proportional gain (kP) 5 6
    0,   // integral gain (kI)
    40,  // derivative gain (kD) 14 27
    3,   // anti windup - 3
    0.5, // small error range, in degrees - 1
    100, // small error range timeout, in milliseconds - 100
    1.5, // large error range, in degrees - 3
    500, // large error range timeout, in milliseconds - 500
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

ModularControl displayHandler(chassis,
                              master,
                              outtake,
                              1); // His name is displayHandler. Please respect
                                  // displayHandler

/**                                                                            \
 * Runs initialization code. This occurs as soon as the program is started.    \
 *                                                                             \
 * All other competition modes are blocked by initialize; it is recommended    \
 * to keep execution time for this mode under a few seconds.                   \
 */

void apr() { auton_one_side(-1, 1); }
void anr() { auton_one_side(-1, -1); }
void apb() { auton_one_side(1, 1); }
void anb() { auton_one_side(1, -1); }
void skills() { auton_one_side(-1, 1, true); }

// It looks nice
rd::Selector selector({
    { "autons_positive_red", apr },

    { "autons_negative_red", anr },

    { "auton_skills", skills },
    { "autons_positive_blue", apb },

    { "autons_negative_blue", anb },
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
    selector.next_auton();
    outtake.initialize();
    // chassis.setPose(
    //     gps.get_position_x() * 39.37, gps.get_position_y() * 39.37, 0);

    //  Initialize chassis and macros
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
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);

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
bool canUpdateMacros;

void displayUpdater(int iteration) {
    if (std::abs(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X)) > 115) {
        if (canUpdateMacros) {
            displayHandler.incrementAddress(
                std::abs(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X)) /
                master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X));
            if (displayHandler.get_active_address() < 1) {
                displayHandler.incrementAddress(
                    macros.size() + (displayHandler.get_active_address() * -1));
            } else if (displayHandler.get_active_address() > macros.size()) {
                displayHandler.incrementAddress(
                    (displayHandler.get_active_address() * -1) + 1);
            }
            displayHandler.updateDisplay(
                macros[displayHandler.get_active_address() - 1]);
            canUpdateMacros = false;
        }
    } else {
        canUpdateMacros = true;
    }
    if (iteration % 10 == 0) { master.clear(); }
    if (iteration % 5 == 0) {
        displayHandler.updateDisplay(
            macros[displayHandler.get_active_address() - 1]);
    }
}

void runMacros() {
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
        displayHandler.activateMacro(
            macros[displayHandler.get_active_address() - 1], isRed);
    }
}

void driveControl() {
    //  Get how far the joysticks are moved
    int leftY = (master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));
    int rightY = (master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y));
    // make it TENK
    chassis.tank(leftY, rightY);
}

void handleEjection() {
    if (colorDetector.get_color() == colorDetector.RED &&
        outtake.get_state() != Outt_States::OFF) {
        outtake.emergency(350, { { 1, -1 }, { 1, 1 }, { 1, 1 } });
    }
}

void handleOuttakeCont() {
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
        if (outtake.get_state() == Outt_States::BOTTOM) {
            outtake.run_at_state(Outt_States::OFF);
        } else {
            outtake.run_at_state(Outt_States::BOTTOM);
        }
    } else if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
        if (outtake.get_state() == Outt_States::TOP) {
            outtake.run_at_state(Outt_States::OFF);
        } else {
            outtake.run_at_state(Outt_States::TOP);
        }
    } else if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) {
        if (outtake.get_state() == Outt_States::MIDDLE) {
            outtake.run_at_state(Outt_States::OFF);
        } else {
            outtake.run_at_state(Outt_States::MIDDLE);
        }
    }
}

void handleControls() {
    handleOuttakeCont();
    // Toggle funny scrapers
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
        scraper.toggle();
    }

    // "Double-Park-Thingy" as I was told
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
        descorer.toggle();
    }
}

void opcontrol() {

    // scraper.toggle();
    //  UpdateDisplay
    displayHandler.updateDisplay(
        macros[displayHandler.get_active_address() - 1]);
    // if (master.get_digital(pros::E_CONTROLLER_DIGITAL_A) &&
    //     master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)) {
    //     autonomous();
    //     return;
    // }

    // Set Brake Mode
    pros::motor_brake_mode_e_t driver_preference_brake =
        pros::E_MOTOR_BRAKE_COAST;

    chassis.setBrakeMode(driver_preference_brake);

    int iteration = 0;
    // watch afshin implode the bot
    while (true) {
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_A) &&
            master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)) {
            anb();
            return;
        }
        displayUpdater(iteration); // Handles controller display
        // runMacros();               // Runs Macros
        driveControl(); // Controls Chassis
        // handleEjection(); // Ejects incorrect color bloaks
        handleControls(); // Handles mechanism controls
        pros::delay(10);  // Timer calculations
        iteration++;
    }
}