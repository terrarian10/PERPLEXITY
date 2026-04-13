#include "main.h"
#include "Outtake.hpp"
#include "airCylinder.hpp"
#include "autons.hpp"
#include "color_sort.hpp"
#include "consts.hpp"
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
pros::MotorGroup left_motors({ -11, 12, -20 }, pros::MotorGearset::blue);
pros::MotorGroup right_motors({ 1, -2, 10 }, pros::MotorGearset::blue);
// Piggyback off of purdues hard work
lemlib::Drivetrain drivetrain(&left_motors,  // left motor group
                              &right_motors, // right motor group
                              11.25,         // 10 inch track width // 13.82?
                              lemlib::Omniwheel::NEW_325,
                              450, // drivetrain rpm is 360
                              8    // horizontal drift is 2 (for now)
);
bool isRed;
pros::Optical optical(30);
ColourDetector colorDetector(optical);
// Initialize the Scraper
AirCylinder scraper('c', false);
AirCylinder middle_scorer('h', false);
ticker& roboHandler() {
    static ticker bot; // one persistent instance
    return bot;
}
AirCylinder descorer_l('a');
pros::IMU imu(14);

pros::Rotation vertOdom(4);

// Literally anything  is better than this method
// Don't touch it it works
pros::Motor outt_1(13, pros::MotorGearset::blue);
pros::Motor outt_2(-3, pros::MotorGearset::blue);
std::vector<pros::Motor> test = { outt_1, outt_2 };
// AirCylinder scoring_cylinder('h');
std::vector<AirCylinder> intake_cylinders = { middle_scorer };
mecha_control outtake_ctrl = {
    { { { { 0, 1 }, { 1, 1, 1 }, { -1, 0 } },
        pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_R1 },
      { { { 0, 1 }, { 1, -0.35, 1 }, { -1, 0 } },
        pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_L1 },
      { { { 0, -1 }, { 1, 1, 1 }, { -1, 0 } },
        pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_L2 },
      { { { 0, 0.8 }, { 1, 0.75, 1 }, { -1, 1 } },
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
lemlib::TrackingWheel vertical(&vertOdom, lemlib::Omniwheel::NEW_2, -3);

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
    21,   // proportional gain (kP) - 28
    0,    // integral gain (kI)
    19.5, // derivative gain (kD) -- 9
    3,    //-48.375, // anti windup3
    1,    // small error range, in inches1
    100,  // small error range timeout, in milliseconds100
    3,    // large error range, in inches3
    500,  // large error range timeout, in milliseconds500
    0     // maximum acceleration (slew)0
);

lemlib::ControllerSettings angular_controller(
    4.75, // kP – start here again or even 4.1
    0,    // kI – keep off for now
    26,   // kD – moderate, not 28
    3,    // 3 anti windup (does nothing until you use I, but fine)
    1,    // 1 small error range (deg)
    100,  // 100 small error timeout (ms)
    3,    // 3 large error range (deg)
    300,  // 300 large error timeout (ms)
    40    // 40 slew – limit acceleration so it doesn't slam
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
void apr() {
    isRed = true;
    quarterAuto(false);
}
void aprLong() {
    isRed = true;
    longGoal(false, left_motors, right_motors);
}
void anbLong() {
    isRed = true;
    longGoal(true, left_motors, right_motors);
}
void anr() {
    isRed = true;
    lowGoal_auto(false);
}
void apr_full() {
    isRed = true;
    halfAuto(false);
}

void anb_full() {
    isRed = false;
    halfAuto(true);
}
void mvfwrd() {
    isRed = false;
    mvfwd();
}
void apb() {
    isRed = false;
    lowGoal_auto(true);
}
void anb() {
    isRed = false;
    quarterAuto(true);
}
void pid_test() { testing_pid(); }

// void skills() { auton_full(true, true); }
void skills() { auton_skills(); }

// It looks nice
rd::Selector selector({
    { "sawp_blue_midgoal", anb_full },
    { "sawp_red_midgoal", apr_full },
    { "quarter_mid_red", apr },
    { "autons_negative_red", anr },
    { "auton_skills", skills },
    { "autons_positive_blue", apb },
    { "quarter_mid_blue", anb },
    { "forward", mvfwrd },

});
void initialize_macros() {
    std::cout << outtake_ctrl.motorHandling.at(4).control;
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

void opcontrol() {
    // chassis.setPose(0, 0, 0); // REMOVE THIS LATER PLEASE
    //  Set Brake Mode
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
            apb();
            return;
        }
        if (iteration % 50 == 0) {

            std::cout << "Outtake Motor TEMP: " << outt_1.get_temperature()
                      << std::endl;
            std::cout << "Motors Left - Drive "
                      << left_motors.get_temperature(0) << " "
                      << left_motors.get_temperature(1) << " "
                      << left_motors.get_temperature(2) << std::endl;
            std::cout << "Motors Right - Dri,ve "
                      << right_motors.get_temperature(0) << " "
                      << right_motors.get_temperature(1) << " "
                      << right_motors.get_temperature(2) << std::endl;
            std::cout << "POSE: " << chassis.getPose().x << " "
                      << chassis.getPose().y << " " << chassis.getPose().theta
                      << std::endl;
        }

        iteration++;
    }
}