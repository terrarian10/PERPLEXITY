#include "commandHandling.hpp"
#include "consts.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/pose.hpp"
#include "zcommands/chassisCommands.hpp"
#include "zcommands/math.hpp"
class descore_macro : public command {
  public:
    explicit descore_macro(virtualController& vc,
                           pros::controller_digital_e_t control,
                           lemlib::Chassis& chassis,
                           Scheduler& scheduler)
        : vc(vc)
        , control(control)
        , chassis(chassis)
        , scheduler(scheduler) {}
    bool run() override {
        auto presses = vc.getNewPress();
        if (std::find(presses.begin(), presses.end(), control) !=
            presses.end()) {
            scheduler.enqueue<true_movepose_c>(
                relativeMove(5, chassis.getPose()),
                chassis,
                poseCFG{ .minSpeed = 50 });
            scheduler.enqueue<true_swing_heading_c>(
                chassis.getPose().theta + 180,
                DriveSide::RIGHT,
                chassis,

                swingCFG{ .maxSpeed = 127,
                          .minSpeed = 0,
                          .timeout = 1000,
                          .dirPath = AngularDirection::CW_CLOCKWISE });
            scheduler.move_to_front();
            scheduler.move_to_front();
        }
        command* current = scheduler.get_current();

        if ((dynamic_cast<true_movepose_c*>(current) ||
             dynamic_cast<true_swing_heading_c*>(current)) ||
            dynamic_cast<true_turnheading_c*>(current) &&
                (std::any_of(
                    vc.joysticks.begin(),
                    vc.joysticks.end(),
                    [](const auto& pair) { return pair.second > 50; }))) {
            scheduler.force_quit_current();
        }
        return true;
    };

  private:
    virtualController& vc;
    pros::controller_digital_e_t control;
    Scheduler& scheduler;
    lemlib::Chassis& chassis;
};