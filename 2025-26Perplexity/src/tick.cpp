#include "tick.hpp"
#include "airCylinder.hpp"
#include "autons.hpp"
#include "commandHandling.hpp"
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
    std::vector<particle> particles{};
    lemlib::Pose oldPose = chassis.getPose();
    std::vector<pros::Distance> distance{};
    std::vector<lemlib::Pose> offsets{};
    float sigma = 2;
    monteConfig localization{ .particles = particles,
                              .chassis = chassis,
                              .oldPose = oldPose,
                              .distance = distance,
                              .offsets = offsets };
    botScheduler().enqueue<populateMCL_c>(localization, chassis.getPose());
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
        tickMCL_c(localization, sigma),
        wait_c(10),
        poseMCL_c(localization, sigma));
}