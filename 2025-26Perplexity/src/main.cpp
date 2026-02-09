#include "main.h"
#include "Outtake.hpp"
#include "airCylinder.hpp"
#include "autons.hpp"
#include "color_sort.hpp"
#include "consts.hpp"
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
#include "tick.hpp"
#include <cstddef>
#include <cstdlib>
#include <iostream>
#include <vector>

// Chassis constructor
pros::MotorGroup left_motors({ 20, -19, -18 }, pros::MotorGearset::blue);
pros::MotorGroup right_motors({ 8, 9, -10 }, pros::MotorGearset::blue);
// Piggyback off of purdues hard work
lemlib::Drivetrain drivetrain(&left_motors,  // left motor group
                              &right_motors, // right motor group
                              11.25,         // 10 inch track width // 13.82?
                              lemlib::Omniwheel::NEW_325,
                              360, // drivetrain rpm is 360
                              8    // horizontal drift is 2 (for now)
);
bool isRed;
pros::Optical optical(30);
ColourDetector colorDetector(optical);
// Initialize the Scraper
AirCylinder scraper('b', false);
AirCylinder middle_scorer('a', false);
ticker& roboHandler() {
    static ticker bot; // one persistent instance
    return bot;
} // Make attacher wirj
// AirCylinder attacher('g');
// Doubleparrk
AirCylinder descorer_l('c');

pros::IMU imu(10);
// pros::Rotation horizOdom(9);
//  pros::Rotation horizOdom2(4);
pros::Rotation vertOdom(4);

// Literally anything  is better than this method
// Don't touch it it works
pros::Motor outt_1(-16, pros::MotorGearset::blue);
pros::Motor outt_2(-15, pros::MotorGearset::blue);
std::vector<pros::Motor> test = { outt_1, outt_2 };
// AirCylinder scoring_cylinder('h');
std::vector<AirCylinder> intake_cylinders = { middle_scorer };
mecha_control outtake_ctrl = {
    { { { { 0, 1 }, { 1, 1, 1 }, { -1, 0 } },
        pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_R1 },
      { { { 0, 1 }, { 1, -0.5, 1 }, { -1, 0 } },
        pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_L1 },
      { { { 0, -1 }, { 1, -1, 1 }, { -1, 0 } },
        pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_L2 },
      { { { 0, 1 }, { 1, 1, 1 }, { -1, 1 } },
        pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_R2 },
      { { { 0, 0 }, { 1, 0, 1 }, { -1, -1 } } } },
    "Outtake"
};

std::vector<single_control> outtake_idle = { { 0, 0 }, { 1, 0 }, { -1, 0 } };
Outtake outtake(test,
                intake_cylinders,
                colorDetector,
                outtake_ctrl,
                outtake_idle,
                12000);

// lemlib::TrackingWheel horizontal(&horizOdom, lemlib::Omniwheel::NEW_2, -1.5);
lemlib::TrackingWheel vertical(&vertOdom, lemlib::Omniwheel::NEW_2, -1.125);

// The sensors are imaginary
lemlib::OdomSensors sensors(
    &vertical, //&vertical,   // vertical tracking wheel 1, set to nullptr
    nullptr,   // vertical tracking wheel 2
    nullptr,
    //&horizontal, // horizontal tracking wheel 1
    nullptr, //&horizontal2, // horizontal tracking wheel 2
    &imu     // imu
);
lemlib::OdomSensors imu_only(
    nullptr,
    nullptr, // vertical tracking wheel 2
    nullptr, // horizontal tracking wheel 1
    nullptr, //&horizontal2, // horizontal tracking wheel 2
    &imu     // imu
);
lemlib::OdomSensors nan_sensor(
    nullptr,
    nullptr, // vertical tracking wheel 2
    nullptr, // horizontal tracking wheel 1
    nullptr, //&horizontal2, // horizontal tracking wheel 2
    nullptr  // imu
);
// Guess and check final boss
// lateral PID controller
lemlib::ControllerSettings lateral_controller(
    28,  // proportional gain (kP) - 15 55 60 65 70 75 80
    0,   // integral gain (kI)
    9,   // derivative gain (kD) -- 9 32 32 33 34 36
    3,   //-48.375, // anti windup3
    1,   // small error range, in inches1
    100, // small error range timeout, in milliseconds100
    3,   // large error range, in inches3
    500, // large error range timeout, in milliseconds500
    20   // maximum acceleration (slew)0
);

lemlib::ControllerSettings angular_controller(
    4.1, // kP – start here again or even 3
    0,   // kI – keep off for now
    26,  // kD – moderate, not 400
    3,   // 3 anti windup (does nothing until you use I, but fine)
    1,   // 1 small error range (deg)
    100, // 100 small error timeout (ms)
    3,   // 3 large error range (deg)
    500, // 300 large error timeout (ms)
    0    // 40 slew – limit acceleration so it doesn't slam
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
void apr() {
    isRed = true;
    auton_one_side(-1, 1);
}
void anr() {
    isRed = true;
    auto_bottomGoal(false);
}
void anr_full() {
    isRed = true;
    auton_full(false, false);
}
void apb_full() {
    isRed = false;
    auton_full(true, false);
}
void apb() {
    isRed = false;
    auto_bottomGoal(true);
}
void anb() {
    isRed = false;
    auton_one_side(1, -1);
}
void pid_test() { testing_pid(); }

// void skills() { auton_full(true, true); }
void skills() { park(); }

// It looks nice
rd::Selector selector({
    { "autons_positive_blue_full", apb_full },
    { "autons_negative_red_full", anr_full },
    { "autons_positive_red", apr },

    { "autons_negative_red", anr },

    { "auton_skills", skills },
    { "autons_positive_blue", apb },

    { "autons_negative_blue", anb },
});
void initialize_macros() {
    std::cout << outtake_ctrl.motorHandling.at(4).control;
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
    outt_2.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
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

// void handleOuttakeCont() {
//     if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) {
//         if (outtake.get_state() == Outt_States::BOTTOM) {
//             outtake.run_at_state(Outt_States::OFF);
//         } else {
//             outtake.run_at_state(Outt_States::BOTTOM);
//         }
//     } else if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) {
//         if (outtake.get_state() == Outt_States::TOP) {
//             outtake.run_at_state(Outt_States::OFF);
//         } else {
//             outtake.run_at_state(Outt_States::TOP);
//             outtake.emergency(120, { { 0, -1 }, { 1, 0 }, { -1, -1 } });
//         }
//     } else if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) {
//         if (outtake.get_state() == Outt_States::MIDDLE) {
//             outtake.run_at_state(Outt_States::OFF);
//         } else {
//             outtake.run_at_state(Outt_States::MIDDLE);
//             outtake.emergency(120, { { 0, -1 }, { 1, 0 }, { -1, -1 } });
//         }
//     } else if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
//         if (outtake.get_state() == Outt_States::HOARD) {
//             outtake.run_at_state(Outt_States::OFF);
//         } else {
//             outtake.run_at_state(Outt_States::HOARD);
//         }
//     }

//     if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
//         outtake.emergency(50, { { 0, 1 }, { 1, 1 }, { -1, -1 } });
//     }
//     if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
//         outtake.emergency(150, { { 0, -1 }, { 1, -1 }, { -1, -1 } });
//     }
// }

void handleControls() {
    // handleOuttakeCont();
    //  Toggle funny scrapers
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
        scraper.toggle();
    }

    // "Double-Park-Thingy" as I was told
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
        descorer_l.toggle();
    }
}

void opcontrol() {

    displayHandler.updateDisplay(
        macros[displayHandler.get_active_address() - 1]);

    // Set Brake Mode
    pros::motor_brake_mode_e_t driver_preference_brake =
        pros::E_MOTOR_BRAKE_COAST;
    roboHandler().command_opcontrol();
    chassis.setBrakeMode(driver_preference_brake);

    int iteration = 0;
    // watch afshin implode the bot
    while (true) {
        roboHandler().tick();
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN) &&
            master.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
            anr();
            return;
        }
        if (iteration % 50 == 0) {
            std::cout << "Outtake Motor Efficiency: " << outt_1.get_efficiency()
                      << std::endl;
            std::cout << "Outtake Motor TEMP: " << outt_1.get_temperature()
                      << std::endl;
            std::cout << "Motors Left - Drive "
                      << left_motors.get_temperature(0) << " "
                      << left_motors.get_temperature(1) << " "
                      << left_motors.get_temperature(2) << std::endl;
            std::cout << "Motors Right - Drive "
                      << right_motors.get_temperature(0) << " "
                      << right_motors.get_temperature(1) << " "
                      << right_motors.get_temperature(2) << std::endl;
        }

        iteration++;
    }
}