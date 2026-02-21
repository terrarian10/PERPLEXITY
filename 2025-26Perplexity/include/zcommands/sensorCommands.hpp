#include "commandHandling.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "main.h"
#include "pros/distance.hpp"
#include "pros/misc.h"

class setPosFromDistance_c : public command {
  public:
    explicit setPosFromDistance_c(pros::Distance distance,
                                  lemlib::Chassis& chassis,
                                  bool setX,
                                  float offset = 5)
        : chassis(chassis)
        , distance(distance)
        , offset(offset)
        , setX(setX) {}
    bool run() override {
        if (setX) {
            if (chassis.getPose().x < 0) {
                chassis.setPose(-(70 - (distance.get() / 25.4 - offset)),
                                chassis.getPose().y,
                                chassis.getPose().theta);
            } else {
                chassis.setPose((70 - (distance.get() / 25.4 - offset)),
                                chassis.getPose().y,
                                chassis.getPose().theta);
            }
        } else {
            if (chassis.getPose().y < 0) {
                chassis.setPose(chassis.getPose().x,
                                -(70 - (distance.get() / 25.4 - offset)),
                                chassis.getPose().theta);
            } else {
                chassis.setPose(chassis.getPose().x,
                                (70 - (distance.get() / 25.4 - offset)),
                                chassis.getPose().theta);
            }
        }
        return true;
    }
    void quit() override {}
    void force_quit() override {}

  private:
    pros::Distance distance;
    lemlib::Chassis& chassis;
    float offset;
    bool setX;
};
class setY_dist_c : public command {
  public:
    explicit setY_dist_c(pros::Distance distance,
                         lemlib::Chassis& chassis,
                         bool isTop,
                         float offset = 7.125)
        : chassis(chassis)
        , distance(distance)
        , offset(offset)
        , isTop(isTop) {}
    bool run() override {

        if (!isTop) {
            chassis.setPose(chassis.getPose().x,
                            -(70 - (distance.get() / 25.4 - offset)),
                            chassis.getPose().theta);
        } else {
            chassis.setPose(chassis.getPose().x,
                            (70 - (distance.get() / 25.4 - offset)),
                            chassis.getPose().theta);
        }

        return true;
    }
    void quit() override {}
    void force_quit() override {}

  private:
    pros::Distance distance;
    lemlib::Chassis& chassis;
    float offset;
    bool isTop;
};