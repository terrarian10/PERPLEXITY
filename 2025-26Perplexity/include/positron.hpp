
#pragma once

// Import unnecessary classes
#include "Outtake.hpp"
#include "airCylinder.hpp"
#include "color_sort.hpp"
#include "consts.h"
#include "lemlib/chassis/chassis.hpp"
#include "modularSubsystem.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/motor_group.hpp"
#include "pros/rtos.hpp"
#include <cstddef>
#include <string>
#include <vector>

enum uses { SCORING, LOADING, BLOCKING };

struct poi {
    float x1;
    float y1;
    uses use;
    float y2 = NULL;
    float x2 = NULL;
};

class perform {
  public:
    virtual ~perform() = default;
    virtual void run();
    virtual void quit();
};

class perform_score {};

class positron {
  public:
    positron(Outtake& outtake,
             lemlib::Chassis& chassis,
             std::vector<poi>& interest)
        : outtake(outtake)
        , chassis(chassis) {
        interests.emplace_back(interest);
    };
    inline void add_intrest(std::vector<poi> interest) {
        interests.emplace_back(interest);
    };

  private:
    Outtake& outtake;
    lemlib::Chassis& chassis;
    std::vector<std::vector<poi>> interests;
    poi closest();
};
