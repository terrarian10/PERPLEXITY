#include "airCylinder.hpp"
#include "commandHandling.hpp"
#include "consts.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/pose.hpp"
#include "main.h"
#include "zcommands/chassisCommands.hpp"
#include "zcommands/math.hpp"
#include "zcommands/mechCommands.hpp"
class descore_macro : public command {
  public:
    explicit descore_macro(virtualController& vc,
                           pros::controller_digital_e_t control,
                           lemlib::Chassis& chassis,
                           AirCylinder& descore,
                           Scheduler& scheduler)
        : vc(vc)
        , control(control)
        , chassis(chassis)
        , descore(descore)
        , scheduler(scheduler) {}
    bool run() override {
        auto presses = vc.getNewPress();
        if (std::find(presses.begin(), presses.end(), control) !=
            presses.end()) {
            scheduler.enqueue<rel_movepose_c>(
                1, chassis, poseCFG{ .minSpeed = 50, .earlyExitRange = 2 });
            scheduler.enqueue<rel_swing_heading_c>(
                120,
                DriveSide::RIGHT,
                chassis,

                swingCFG{ .maxSpeed = 127,
                          .minSpeed = 100,
                          .timeout = 1000,
                          .dirPath = AngularDirection::CW_CLOCKWISE });
            scheduler.enqueue<rel_turnheading_c>(
                61,
                chassis,
                poseCFG{ .minSpeed = 100, .maxSpeed = 127, .timeout = 1000 });
            scheduler.enqueue<rel_movepose_c>(
                60, chassis, poseCFG{ .minSpeed = 127 });
            scheduler.enqueue<switchPneu_c>(descore, false);
            scheduler.enqueue<rel_turnheading_c>(
                5,
                chassis,
                poseCFG{ .minSpeed = 100, .maxSpeed = 127, .timeout = 1000 });
            scheduler.enqueue<rel_movepose_c>(
                30, chassis, poseCFG{ .minSpeed = 127 });
            scheduler.enqueue<switchPneu_c>(descore, true);
            scheduler.enqueue<rel_movepose_c>(
                60, chassis, poseCFG{ .minSpeed = 127 });
            scheduler.enqueue<switchPneu_c>(descore, false);
            scheduler.enqueue<rel_movepose_c>(
                60, chassis, poseCFG{ .minSpeed = 127 });
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
    lemlib::Chassis& chassis;
};