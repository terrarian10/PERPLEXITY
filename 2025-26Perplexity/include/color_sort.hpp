/**
 *
 *
 *
 */

#pragma once
#include "pros/optical.hpp"
#include <functional>

class ColourDetector : public pros::Optical {
  public:
    enum Colour { RED, BLUE, NONE };
    ColourDetector(
        pros::Optical& sensor,
        std::function<bool(pros::Optical&)> detect_red =
            [](pros::Optical& optical) -> bool {
            double hue = optical.get_hue();
            return hue > 0 && hue < 30 || hue > 340 && hue < 0;
        },
        std::function<bool(pros::Optical&)> detect_blue =
            [](pros::Optical& optical) -> bool {
            double hue = optical.get_hue();
            return hue > 162 && hue < 290;
        },
        std::function<bool(pros::Optical&)> detect_none =
            [](pros::Optical& optical) -> bool {
            return optical.get_proximity() < 25;
        })
        : pros::Optical(sensor)
        , detect_red(detect_red)
        , detect_blue(detect_blue)
        , detect_none(detect_none) {}

    Colour get_color() {
        if (detect_none(*this)) {
            return NONE;
        } else if (detect_red(*this)) {
            return RED;
        } else if (detect_blue(*this)) {
            return BLUE;
        } else {
            return NONE;
        }
    }

  private:
    std::function<bool(pros::Optical&)> detect_red;
    std::function<bool(pros::Optical&)> detect_blue;
    std::function<bool(pros::Optical&)> detect_none;
};