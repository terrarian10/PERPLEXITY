#include "commandHandling.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/pose.hpp"
#include "main.h"
#include "pros/misc.h"
struct point {
    float x;
    float y;
};

class tank_c : public command {
  public:
    explicit tank_c(lemlib::Chassis& chassis, virtualController& vc)
        : chassis(chassis)
        , vc(vc) {};
    bool run() override {
        chassis.tank(vc.get_joystick(pros::E_CONTROLLER_ANALOG_LEFT_Y),
                     vc.get_joystick(pros::E_CONTROLLER_ANALOG_RIGHT_Y));
        return true;
    };

  private:
    lemlib::Chassis& chassis;
    virtualController& vc;
};

class setPose_c : public command {
  public:
    explicit setPose_c(lemlib::Pose pose)
        : pose(pose) {};
    bool run() override {
        pose.theta = pose.theta > 9000 ? chassis.getPose().theta : pose.theta;
        chassis.setPose(pose);
        std::cout << chassis.getPose().x;
        return true;
    }
    void quit() override {}
    void force_quit() override {}

  private:
    lemlib::Pose pose;
};
class move_point_c : public command {
  public:
    explicit move_point_c(float x,
                          float y,
                          lemlib::Chassis& chassis,
                          poseCFG config = { false, 40, 127, 5000 })
        : chassis(chassis)
        , x(x)
        , y(y)
        , config(config)
        , started(false) {};
    bool run() override {
        if (!started) {
            chassis.moveToPoint(x,
                                y,
                                config.timeout,
                                {
                                    .forwards = !config.reversed,
                                    .maxSpeed = float(config.maxSpeed),

                                    .minSpeed = float(config.minSpeed),
                                    .earlyExitRange = config.earlyExitRange,

                                });
            started = true;
            return false;
        }
        return !chassis.isInMotion();
        // std::cout << chassis.getPose().x << " " << chassis.getPose().y
        //           << "--POSITION" << "\n";
    }

    void force_quit() override { chassis.cancelMotion(); }

  private:
    lemlib::Chassis& chassis;
    float x;
    float y;
    poseCFG config;
    bool started;
};
class movepoint_c : public command {
  public:
    explicit movepoint_c(point target,
                         lemlib::Chassis& chassis,
                         float mx,
                         float my,
                         poseCFG config = { false, 0, 127, 5000 })
        : chassis(chassis)
        , target(target)
        , mx(mx)
        , my(my)
        , config(config)
        , started(false) {};
    bool run() override {
        if (!started) {
            chassis.moveToPoint(target.x * mx / 2.54,
                                target.y * my / 2.54,
                                config.timeout,
                                {
                                    .forwards = !config.reversed,
                                    .maxSpeed = float(config.maxSpeed),

                                    .minSpeed = float(config.minSpeed),
                                    .earlyExitRange = config.earlyExitRange,

                                });
            started = true;
            return false;
        }
        return !chassis.isInMotion();
        // std::cout << chassis.getPose().x << " " << chassis.getPose().y
        //           << "--POSITION" << "\n";
    }

    void force_quit() override { chassis.cancelMotion(); }

  private:
    lemlib::Chassis& chassis;
    point target;
    float mx;
    float my;
    poseCFG config;
    bool started;
};
class move_pose_c : public command {
  public:
    explicit move_pose_c(lemlib::Pose pose,
                         lemlib::Chassis& chassis,
                         poseCFG config = { false, 40, 127, 5000 })
        : chassis(chassis)
        , pose(pose)
        , config(config)
        , started(false) {};
    bool run() override {
        if (!started) {
            chassis.moveToPose(pose.x,
                               pose.y,
                               pose.theta,
                               config.timeout,
                               {
                                   .forwards = !config.reversed,
                                   .horizontalDrift = 8,
                                   .maxSpeed = float(config.maxSpeed),

                                   .minSpeed = float(config.minSpeed),
                                   .earlyExitRange = config.earlyExitRange,

                               });
            started = true;
            std::cout << "GOAL X: " << pose.x << " Y: " << pose.y
                      << "ROT:" << pose.theta << "\n";

            return false;
        }
        return !chassis.isInMotion();
        // std::cout << chassis.getPose().x << " " << chassis.getPose().y
        //           << "--POSITION" << "\n";
    }

    void force_quit() override { chassis.cancelMotion(); }

  private:
    lemlib::Chassis& chassis;
    lemlib::Pose pose;
    poseCFG config;
    bool started;
};
class movepose_c : public command {
  public:
    explicit movepose_c(lemlib::Pose pose,
                        lemlib::Chassis& chassis,
                        float mx,
                        float my,
                        poseCFG config = { false, 0, 127, 5000 })
        : chassis(chassis)
        , pose(pose)
        , config(config)
        , started(false)
        , mx(mx)
        , my(my) {};
    bool run() override {
        if (!started) {
            h = pose.theta;
            // Mirror across Y-axis (left ↔ right)
            if (mx != -1) {
                h = std::fmod(360.0 - h, 360.0);
                if (h < 0) h += 360.0;
            }

            // Mirror across X-axis (top ↔ bottom)
            if (my != +1) {
                h = std::fmod(180.0 - h, 360.0);
                if (h < 0) h += 360.0;
            }
            chassis.moveToPose(pose.x / 2.54,
                               pose.y / 2.54,
                               h,
                               config.timeout,
                               {
                                   .forwards = !config.reversed,
                                   .horizontalDrift = 8,
                                   .maxSpeed = float(config.maxSpeed),

                                   .minSpeed = float(config.minSpeed),
                                   .earlyExitRange = config.earlyExitRange,

                               });
            started = true;
            std::cout << "GOAL X: " << pose.x << " Y: " << pose.y
                      << "ROT:" << pose.theta << "\n";

            return false;
        }
        return !chassis.isInMotion();
        // std::cout << chassis.getPose().x << " " << chassis.getPose().y
        //           << "--POSITION" << "\n";
    }

    void force_quit() override { chassis.cancelMotion(); }

  private:
    lemlib::Chassis& chassis;
    lemlib::Pose pose;
    poseCFG config;
    bool started;
    float mx;
    float my;
    float h;
};
class turn_heading_c : public command {
  public:
    explicit turn_heading_c(int dir,
                            lemlib::Chassis& chassis,
                            poseCFG config = { false, 40, 127, 5000 })
        : chassis(chassis)
        , dir(dir)
        , config(config)
        , started(false) {};
    bool run() override {
        if (!started) {
            chassis.turnToHeading(dir,
                                  config.timeout,
                                  {
                                      .maxSpeed = config.maxSpeed,
                                      .minSpeed = config.minSpeed,
                                      .earlyExitRange = config.earlyExitRange,
                                  });
            started = true;
            return false;
        }
        return !chassis.isInMotion();
        // std::cout << chassis.getPose().x << " " << chassis.getPose().y
        //           << "--POSITION" << "\n";
    }

    void force_quit() override { chassis.cancelMotion(); }

  private:
    lemlib::Chassis& chassis;
    int dir;
    poseCFG config;
    bool started;
};
class turnheading_c : public command {
  public:
    explicit turnheading_c(int dir,
                           lemlib::Chassis& chassis,
                           float mx,
                           float my,
                           poseCFG config = { false, 40, 127, 5000 })
        : chassis(chassis)
        , dir(dir)
        , mx(mx)
        , my(my)

        , config(config)
        , started(false) {};
    bool run() override {
        if (!started) {
            // Mirror across Y-axis (left ↔ right)
            if (mx != -1) {
                dir = std::fmod(360.0 - dir, 360.0);
                if (dir < 0) dir += 360.0;
            }

            // Mirror across X-axis (top ↔ bottom)
            if (my != +1) {
                dir = std::fmod(180.0 - dir, 360.0);
                if (dir < 0) dir += 360.0;
            }
            if (360 > dir) { dir -= 360; }
            chassis.turnToHeading(dir,
                                  config.timeout,
                                  {
                                      .maxSpeed = config.maxSpeed,
                                      .minSpeed = config.minSpeed,
                                      .earlyExitRange = config.earlyExitRange,
                                  });
            started = true;
            return false;
        }
        return !chassis.isInMotion();
        // std::cout << chassis.getPose().x << " " << chassis.getPose().y
        //           << "--POSITION" << "\n";
    }

    void force_quit() override { chassis.cancelMotion(); }

  private:
    lemlib::Chassis& chassis;
    int dir;
    poseCFG config;
    bool started;
    float mx;
    float my;
};