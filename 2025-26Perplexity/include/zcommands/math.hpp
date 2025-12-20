#include "lemlib/pose.hpp"
inline bool isInRoughPos(lemlib::Pose pose,
                         lemlib::Pose target,
                         lemlib::Pose tolerance) {
    return (pose.x - target.x < tolerance.x &&
            pose.y - target.y < tolerance.y &&
            pose.theta - target.theta < tolerance.theta);
}