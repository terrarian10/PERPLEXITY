#include "lemlib/pose.hpp"
#include <optional>
struct distanceData {
    std::optional<float> forward1;
    std::optional<float> forward2;
    std::optional<float> left1;
    std::optional<float> left2;
    std::optional<float> right1;
    std::optional<float> right2;
    std::optional<float> back1;
    std::optional<float> back2;
};
struct particleData {
    distanceData distSenseData;
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

struct sensorWeights {
    float distWeight;
    float obsWeight;
};