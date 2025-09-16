/**
 * @file pathing.hpp
 * @author Gabriel Hein
 * @brief Path handler
 * @version 0.1
 * @date 2025-09-08
 *
 * @copyright Copyright (c) 2025
 *
 */
#include "lemlib/api.hpp"
#include "lemlib/pose.hpp"
#include <numbers>
#include <vector>

struct m_drive {
    int left;
    int right;
};

struct Pose {
    double x;
    double y;
    double theta; // radians (0 = +x axis -- 90, CounterClockWise positive)
};

inline m_drive Pathing(lemlib::Pose bot_loc,
                       lemlib::Pose wanted_pose,
                       m_drive h) {

};
