#include "tick.hpp"
#include "airCylinder.hpp"
#include "autons.hpp"
#include "commandHandling.hpp"
#include "lemlib/pose.hpp"
#include "main.h"
#include "pros/distance.hpp"
#include "pros/misc.h"
#include "zcommands/macroCommands.hpp"
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

    chassis.setPose({ -111.8, 38_cm, 0 });
    static lemlib::Pose oldPose = chassis.getPose();

    static std::array<pros::Distance, 4> distance = { pros::Distance(11),
                                                      pros::Distance(13),
                                                      pros::Distance(6),
                                                      pros::Distance(1) };

    static std::vector<simDistanceSensor> offsets{
        { lemlib::Pose{ 3, 7.125, 0 }, 0 },
        { lemlib::Pose(-13.5_cm, 11.45_cm, -90), 0 },
        { lemlib::Pose{ 11.5_cm, -12_cm, 180 }, 0 },
        { lemlib::Pose(13.5_cm, 11.45_cm, 90), 0 },
    };
    float sigma = 2;
    static monteConfig localization{ .particles = particles,
                                     .chassis = chassis,
                                     .oldPose = oldPose,
                                     .distance = distance,
                                     .offsets = offsets };

    //botScheduler().enqueue<populateMCL_c>(localization, chassis.getPose());

    // botScheduler().enqueue<poseMCL_c>(localization, sigma);
    std::cout << "INITIAL POSE: " << chassis.getPose().x << " "
              << chassis.getPose().y << " " << chassis.getPose().theta
              << std::endl;
    // botScheduler().enqueue<populateMCL_c>(localization, chassis.getPose());
    botScheduler().enqueue<group_repeat_cmd>(
        update_controller_c(mainVirutal, master),
        tank_c(chassis, mainVirutal),
        update_mech_state(outtake, outtake_ctrl, mainVirutal),
        update_mech(outtake),
        update_pneu(airs, pneuCtrl, mainVirutal),
        // descore_macro(mainVirutal,
        //               pros::E_CONTROLLER_DIGITAL_LEFT,
        //               chassis,
        //               botScheduler()),
        //tickMCL_c(localization, sigma),
        wait_c(10)
        // update_controller_c(mainVirutal, master),
        // tank_c(chassis, mainVirutal),
        // update_mech_state(outtake, outtake_ctrl, mainVirutal),
        // update_mech(outtake),
        // update_pneu(airs, pneuCtrl, mainVirutal),
        // // tickMCL_c(localization, sigma),
        // wait_c(10),
        // update_controller_c(mainVirutal, master),
        // tank_c(chassis, mainVirutal),
        // update_mech_state(outtake, outtake_ctrl, mainVirutal),
        // update_mech(outtake),
        // update_pneu(airs, pneuCtrl, mainVirutal),
        // // poseMCL_c(localization, sigma),
        // wait_c(10)
    );
}