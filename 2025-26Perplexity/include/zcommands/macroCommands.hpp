#include "airCylinder.hpp"
#include "commandHandling.hpp"
#include "consts.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/pose.hpp"
#include "main.h"
#include "pros/misc.h"
#include "pros/motor_group.hpp"
#include "zcommands/chassisCommands.hpp"
#include "zcommands/math.hpp"
#include "zcommands/mechCommands.hpp"
class descore_macro : public command {
  public:
    explicit descore_macro(virtualController& vc,
                           pros::controller_digital_e_t control,
                           pros::MotorGroup& left,
                           pros::MotorGroup& right,
                           AirCylinder& descore,
                           Scheduler& scheduler)
        : vc(vc)
        , control(control)
        , left(left)
        , right(right)

        , descore(descore)
        , scheduler(scheduler) {}
    bool run() override {
        auto presses = vc.getNewPress();
        if (std::find(presses.begin(), presses.end(), control) !=
            presses.end()) {
            vc.clearNewPress();
            scheduler.enqueue<direct_dt_c>(127, 127, left, right);
            scheduler.enqueue<wait_c>(190);
            scheduler.enqueue<direct_dt_c>(127, -80, left, right);
            scheduler.enqueue<wait_c>(125);
            scheduler.enqueue<direct_dt_c>(-127, -70, left, right);
            scheduler.enqueue<wait_c>(150);
            scheduler.enqueue<switchPneu_c>(descorer_l, false);
            scheduler.enqueue<direct_dt_c>(-127, -70, left, right);
            scheduler.enqueue<wait_c>(250);
            scheduler.enqueue<direct_dt_c>(-127, -90, left, right);
            scheduler.enqueue<wait_c>(100);
            scheduler.enqueue<direct_dt_c>(0, -127, left, right);
            scheduler.enqueue<wait_c>(100);
            scheduler.enqueue<direct_dt_c>(-127, -127, left, right);
            scheduler.enqueue<wait_c>(60);
            scheduler.enqueue<direct_dt_c>(0, 0, left, right);
            scheduler.move_to_front();
            scheduler.move_to_front();

            scheduler.move_to_front();
            scheduler.move_to_front();
            scheduler.move_to_front();
            scheduler.move_to_front();
            scheduler.move_to_front();
            scheduler.move_to_front();
            scheduler.move_to_front();
            scheduler.move_to_front();
            scheduler.move_to_front();
            scheduler.move_to_front();
            scheduler.move_to_front();
            scheduler.move_to_front();
            scheduler.move_to_front();
            scheduler.move_to_front();
            scheduler.move_to_front();
        }
        // command* current = scheduler.get_current();
        // if (std::any_of(vc.joysticks.begin(),
        //                 vc.joysticks.end(),
        //                 [](const auto& pair) { return pair.second > 50; })) {
        //     while (!dynamic_cast<group_repeat_cmd*>(scheduler.get_current()))
        //     {
        //         scheduler.force_quit_current();
        //     }
        // }

        return true;
    };

  private:
    virtualController& vc;
    pros::controller_digital_e_t control;
    Scheduler& scheduler;
    AirCylinder& descore;
    pros::MotorGroup& left;
    pros::MotorGroup& right;
};
class descore_macro_2 : public command {
  public:
    explicit descore_macro_2(virtualController& vc,
                             pros::controller_digital_e_t control,
                             pros::MotorGroup& left,
                             pros::MotorGroup& right,
                             AirCylinder& descore,
                             Scheduler& scheduler,
                             bool override = false)
        : vc(vc)
        , control(control)
        , left(left)
        , right(right)

        , descore(descore)
        , scheduler(scheduler) {}
    bool run() override {
        if (vc.get_button(control)) {
            vc.set_button(control, false);
            vc.addNewPress(pros::E_CONTROLLER_DIGITAL_UP);
            scheduler.enqueue<descore_macro>(vc,
                                             pros::E_CONTROLLER_DIGITAL_UP,
                                             left,
                                             right,
                                             descorer_l,
                                             scheduler);
            scheduler.enqueue<direct_dt_c>(-80, -127, left, right);
            scheduler.enqueue<wait_c>(25);
            scheduler.enqueue<switchPneu_c>(descorer_l, true);
            scheduler.enqueue<direct_dt_c>(-70, -127, left, right);
            scheduler.enqueue<wait_c>(250);
            scheduler.enqueue<direct_dt_c>(0, 0, left, right);

            scheduler.move_to_front();
            scheduler.move_to_front();
            scheduler.move_to_front();
            scheduler.move_to_front();

            scheduler.move_to_front();
            scheduler.move_to_front();
            scheduler.enqueue<descore_macro>(vc,
                                             pros::E_CONTROLLER_DIGITAL_UP,
                                             left,
                                             right,
                                             descorer_l,
                                             scheduler);
            scheduler.move_to_front();
        }
        // command* current = scheduler.get_current();
        // if (std::any_of(vc.joysticks.begin(),
        //                 vc.joysticks.end(),
        //                 [](const auto& pair) { return pair.second > 50; })) {
        //     while (!dynamic_cast<group_repeat_cmd*>(scheduler.get_current()))
        //     {
        //         scheduler.force_quit_current();
        //     }
        // }

        return true;
    };

  private:
    virtualController& vc;
    pros::controller_digital_e_t control;
    Scheduler& scheduler;
    AirCylinder& descore;
    pros::MotorGroup& left;
    pros::MotorGroup& right;
};
class descore_macro_3 : public command {
  public:
    explicit descore_macro_3(virtualController& vc,
                             pros::controller_digital_e_t control,
                             pros::MotorGroup& left,
                             pros::MotorGroup& right,
                             AirCylinder& descore,
                             Scheduler& scheduler)
        : vc(vc)
        , control(control)
        , left(left)
        , right(right)

        , descore(descore)
        , scheduler(scheduler) {}
    bool run() override {
        if (vc.get_button(control)) {
            // vc.set_button(pros::E_CONTROLLER_DIGITAL_LEFT, true);
            // scheduler.enqueue<descore_macro_2>(vc,
            //                                    pros::E_CONTROLLER_DIGITAL_LEFT,
            //                                    left,
            //                                    right,
            //                                    descorer_l,
            //                                    scheduler);
            scheduler.enqueue<direct_dt_c>(-127, -100, left, right);
            scheduler.enqueue<wait_c>(25);
            scheduler.enqueue<switchPneu_c>(descorer_l, false);
            scheduler.enqueue<direct_dt_c>(-127, -90, left, right);
            scheduler.enqueue<wait_c>(25);
            scheduler.enqueue<switchPneu_c>(descorer_l, false);
            scheduler.enqueue<direct_dt_c>(-127, -90, left, right);
            scheduler.enqueue<wait_c>(150);
            scheduler.move_to_front();
            scheduler.move_to_front();
            scheduler.move_to_front();
            scheduler.move_to_front();

            scheduler.move_to_front();
            scheduler.move_to_front();

            scheduler.move_to_front();
            // START MACRO 2

            vc.set_button(control, false);
            vc.addNewPress(pros::E_CONTROLLER_DIGITAL_UP);
            scheduler.enqueue<descore_macro>(vc,
                                             pros::E_CONTROLLER_DIGITAL_UP,
                                             left,
                                             right,
                                             descorer_l,
                                             scheduler);
            scheduler.enqueue<direct_dt_c>(-80, -127, left, right);
            scheduler.enqueue<wait_c>(80);
            scheduler.enqueue<switchPneu_c>(descorer_l, true);
            scheduler.enqueue<direct_dt_c>(-120, -127, left, right);
            scheduler.enqueue<wait_c>(250);
            scheduler.enqueue<direct_dt_c>(0, 0, left, right);

            scheduler.move_to_front();
            scheduler.move_to_front();
            scheduler.move_to_front();
            scheduler.move_to_front();
            scheduler.move_to_front();

            scheduler.move_to_front();
            scheduler.move_to_front();

            // END MACRO 2
        }
        // command* current = scheduler.get_current();
        // if (std::any_of(vc.joysticks.begin(),
        //                 vc.joysticks.end(),
        //                 [](const auto& pair) { return pair.second > 50; })) {
        //     while (!dynamic_cast<group_repeat_cmd*>(scheduler.get_current()))
        //     {
        //         scheduler.force_quit_current();
        //     }
        // }

        return true;
    };

  private:
    virtualController& vc;
    pros::controller_digital_e_t control;
    Scheduler& scheduler;
    AirCylinder& descore;
    pros::MotorGroup& left;
    pros::MotorGroup& right;
};