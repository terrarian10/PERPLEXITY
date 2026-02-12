#include "tick.hpp"
#include "airCylinder.hpp"
#include "autons.hpp"
#include "commandHandling.hpp"
#include "lemlib/pose.hpp"
#include "main.h"
#include "pros/distance.hpp"
#include "pros/misc.h"
#include "zcommands/chassisCommands.hpp"
#include "zcommands/mechCommands.hpp"
#include "zcommands/virtualCommands.hpp"
#include "zposition/zcommands.hpp"
#include <vector>
void ticker::tick() { botScheduler().tick(); }
void ticker::command_opcontrol() {
    std::vector<AirCylinder*> airs = { &descorer_l, &scraper };
    std::vector<pros::controller_digital_e_t> pneuCtrl = {
        pros::E_CONTROLLER_DIGITAL_Y, pros::E_CONTROLLER_DIGITAL_RIGHT
    };
    static std::vector<particle> particles{};
    static lemlib::Pose oldPose = chassis.getPose();
    chassis.setPose({ -100, -100, 0 });
    static std::array<pros::Distance, 4> distance = { pros::Distance(11),
                                                      pros::Distance(13),
                                                      pros::Distance(6),
                                                      pros::Distance(1) };

    static std::vector<simDistanceSensor> offsets{
        { lemlib::Pose{ -9.8, 18.5, 0 }, 0 },
        { lemlib::Pose(-13.5, 11.45, -90), 0 },
        { lemlib::Pose{ 11.5, -12, 180 }, 0 },
        { lemlib::Pose(13.5, 11.45, 90), 0 },
    };
    float sigma = 2;
    static monteConfig localization{ .particles = particles,
                                     .chassis = chassis,
                                     .oldPose = oldPose,
                                     .distance = distance,
                                     .offsets = offsets };
    botScheduler().enqueue<populateMCL_c>(localization,
                                          lemlib::Pose{ -100, -100, 0 });
    // botScheduler().enqueue<populateMCL_c>(localization, chassis.getPose());
    botScheduler().enqueue<group_repeat_cmd>(
        update_controller_c(mainVirutal, master),
        tank_c(chassis, mainVirutal),
        update_mech_state(outtake, outtake_ctrl, mainVirutal),
        update_mech(outtake),
        update_pneu(airs, pneuCtrl, mainVirutal),
        tickMCL_c(localization, sigma),
        wait_c(10),
        update_controller_c(mainVirutal, master),
        tank_c(chassis, mainVirutal),
        update_mech_state(outtake, outtake_ctrl, mainVirutal),
        update_mech(outtake),
        update_pneu(airs, pneuCtrl, mainVirutal),
        tickMCL_c(localization, sigma),
        wait_c(10),
        update_controller_c(mainVirutal, master),
        tank_c(chassis, mainVirutal),
        update_mech_state(outtake, outtake_ctrl, mainVirutal),
        update_mech(outtake),
        update_pneu(airs, pneuCtrl, mainVirutal),
        poseMCL_c(localization, sigma),
        wait_c(10));
}