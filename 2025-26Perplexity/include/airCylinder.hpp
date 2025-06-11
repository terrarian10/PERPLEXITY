/**
 * @file airCylinder.hpp
 * @author Dylan Shah
 * @brief A class to control a pneumatic cylinder and save the state of the
 * cylinder.
 * @version 1.1
 * @date 2025-01-05
 *
 * @copyright Copyright (c) 2025
 *
 */

#pragma once
#include "pros/adi.hpp"
#include <cstdint>

class AirCylinder {
  public:
    /**
     * @brief Construct a new smart Cylinder object
     *
     * @param port
             The ADI port number (from 1-8, 'a'-'h', 'A'-'H') to configure
     * @param value
             The value that the piston STARTS WITH
     */
    AirCylinder(std::uint8_t port, bool value = false)
        : ADIport(port, value)
        , value(value) {}

    /**
     * @brief Set the value object
     *
     * @param value
     */
    void set_value(bool value) {
        this->value = value;
        ADIport.set_value(value);
    }
    /**
     * @brief Toggles the pneumatic cylinder to the other state
     *
     */
    void toggle(void) { this->set_value(!this->value); }
    /**
     * @brief Get the state of the air cylinder
     *
     * @return true The cylinder is extended
     * @return false The cylinder is not extended
     */
    bool get_value(void) { return this->value; }

    void inline extend(void) { set_value(true); }
    void inline retract(void) { set_value(false); }

  private:
    bool value;
    pros::adi::DigitalOut ADIport;
};
