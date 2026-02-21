

#include "commandHandling.hpp"
#include "lemlib/chassis/chassis.hpp"
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
            scheduler.enqueue<movepose_c>(
                relativeMove(4_cm, chassis.getPose()), chassis, 1, 1);
            scheduler.enqueue<swing_heading_c>(
                chassis.getPose().theta + 180,
                DriveSide::RIGHT,
                chassis,
                1,
                1,
                swingCFG{ .maxSpeed = 127,
                          .minSpeed = 50,
                          .timeout = 1000,
                          .dirPath = AngularDirection::CW_CLOCKWISE });
            scheduler.move_to_front();
            scheduler.move_to_front();
        }

        return true;
    };

  private:
    virtualController& vc;
    pros::controller_digital_e_t control;
    Scheduler& scheduler;
    lemlib::Chassis& chassis;
};