#pragma once
#include "consts.hpp"
#include "lemlib/pose.hpp"
#include <cmath>
#include <iostream>
#include <numbers>
#include <vector>
inline bool isInRoughPos(lemlib::Pose pose,
                         lemlib::Pose target,
                         lemlib::Pose tolerance) {
    return (std::abs(pose.x - target.x) < tolerance.x &&
            std::abs(pose.y - target.y) < tolerance.y &&
            std::abs(pose.theta - target.theta) < tolerance.theta);
}

inline float raycast(lemlib::Pose pos, box field) {
    // field_size = {field.minX, field.maxX, field.minY, field.maxY}

    const float x0 = pos.x;
    const float y0 = pos.y;

    // Convert theta to a direction vector.
    // If LemLib theta = 0 means "pointing up" and increases CCW/CW differently,
    // you can adjust this, but the structure stays the same.
    const float rad = (90.0f - pos.theta) * (float)M_PI / 180.0f;

    // Common convention: theta=0 -> +X, theta=90 -> +Y
    // If your robot convention differs, swap/negate accordingly.
    const float dx = std::cos(rad);
    const float dy = std::sin(rad);

    const float EPS = 1e-6f;
    float bestT = std::numeric_limits<float>::infinity();

    auto tryT = [&](float t) {
        if (t < 0) return;
        if (t < bestT) bestT = t;
    };

    // Intersect with x = field.minX and x = field.maxX
    if (std::fabs(dx) > EPS) {
        float t = (field.minX - x0) / dx;
        float y = y0 + t * dy;
        if (t >= 0 && y >= field.minY - EPS && y <= field.maxY + EPS) tryT(t);

        t = (field.maxX - x0) / dx;
        y = y0 + t * dy;
        if (t >= 0 && y >= field.minY - EPS && y <= field.maxY + EPS) tryT(t);
    }

    // Intersect with y = field.minY and y = field.maxY
    if (std::fabs(dy) > EPS) {
        float t = (field.minY - y0) / dy;
        float x = x0 + t * dx;
        if (t >= 0 && x >= field.minX - EPS && x <= field.maxX + EPS) tryT(t);

        t = (field.maxY - y0) / dy;
        x = x0 + t * dx;
        if (t >= 0 && x >= field.minX - EPS && x <= field.maxX + EPS) tryT(t);
    }

    // If no hit (shouldn't happen unless field_size is bad), return -1
    if (!std::isfinite(bestT)) return -1.0f;

    // Since (dx,dy) is unit length, bestT is the distance.
    return bestT;
}
static inline float wrapDeg(float deg) {
    while (deg <= -180.0f)
        deg += 360.0f;
    while (deg > 180.0f)
        deg -= 360.0f;
    return deg;
}

inline static float vexRaycast(lemlib::Pose pos) {
    return raycast(pos, box{ -182.88, -182.88, 182.88, 182.88 });
}

inline lemlib::Pose relativeMove(double distance, lemlib::Pose chassisPose) {
    std::cerr << "in function\v";
    float rad = std::numbers::pi / 180;
    return lemlib::Pose(
        chassisPose.x * 2.54 + distance * std::sin(chassisPose.theta * rad),
        chassisPose.y * 2.54 + distance * std::cos(chassisPose.theta * rad),
        chassisPose.theta);
}