#include "zcommands/math.hpp"

inline float ghostDistance(lemlib::Pose pos, lemlib::Pose offset) {

    lemlib::Pose sensorPose = pos + offset;
    float distance =
        raycast(sensorPose, box{ -182.88, -182.88, 182.88, 182.88 });

    return distance;
}