#include "lemlib/pose.hpp"
#include "zcommands/math.hpp"
#include <cstddef>
#include <optional>
#include <variant>
#include <vector>

struct particleData {
    std::optional<float> forward1;
    std::optional<float> forward2;
    std::optional<float> left1;
    std::optional<float> left2;
    std::optional<float> right1;
    std::optional<float> right2;
    std::optional<float> back1;
    std::optional<float> back2;
};
// This is a happy little particle
struct particle {
    lemlib::Pose pose;
    particleData sensorData;
};

struct simDistSensor {
    lemlib::Pose offset;
};

struct botDistanceSensors {
    std::optional<simDistSensor> forward1;
    std::optional<simDistSensor> forward2;
    std::optional<simDistSensor> left1;
    std::optional<simDistSensor> left2;
    std::optional<simDistSensor> right1;
    std::optional<simDistSensor> right2;
    std::optional<simDistSensor> back1;
    std::optional<simDistSensor> back2;
};
inline std::optional<float> raycastIfPresent(
    const std::optional<simDistSensor>& sensor,
    const lemlib::Pose& pose) {
    if (!sensor) return std::nullopt;

    return raycast(sensor->offset + pose,
                   box{ -182.88, -182.88, 182.88, 182.88 });
}

inline particleData buildParticleData(const botDistanceSensors& sensors,
                                      const lemlib::Pose& pose) {
    return {
        .forward1 = raycastIfPresent(sensors.forward1, pose),
        .forward2 = raycastIfPresent(sensors.forward2, pose),
        .left1 = raycastIfPresent(sensors.left1, pose),
        .left2 = raycastIfPresent(sensors.left2, pose),
        .right1 = raycastIfPresent(sensors.right1, pose),
        .right2 = raycastIfPresent(sensors.right2, pose),
        .back1 = raycastIfPresent(sensors.back1, pose),
        .back2 = raycastIfPresent(sensors.back2, pose),
    };
}

inline particle createParticle(lemlib::Pose botPose,
                               botDistanceSensors distanceSensors) {

    return particle{ .pose = botPose,
                     .sensorData =
                         buildParticleData(distanceSensors, botPose) };
};