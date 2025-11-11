/**
 * @file modularSubsystem.hpp
 * @author Gabriel Hein
 * @brief Subsystem handler
 * @version 0.1
 * @date 2025-07-13
 *
 * @copyright Copyright (c) 2025
 *
 */

#pragma once

// Import unnecessary classes
#include "color_sort.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/motor_group.hpp"
#include "pros/motors.hpp"
#include "pros/optical.hpp"
#include "pros/rtos.hpp"
#include "pros/vision.hpp"
#include <string>
#include <vector>

struct subCFG {
    int baseSpeed = 12000;
};

class subsystem {
  public:
    virtual ~subsystem() = default;
    virtual void initialize() {}
    virtual void run() {}
    virtual void quit() {}

    virtual void suspend_time(int) {}
    virtual void suspend() {}
    virtual void resume() {}

    virtual std::string log() { return ""; }

  protected:
    subCFG config;
    subCFG& cfg() { return config; }
    const subCFG& cfg() const { return config; }
};